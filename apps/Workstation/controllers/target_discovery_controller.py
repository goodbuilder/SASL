"""Target discovery/application commands with bounded thread ownership.

Injected operations are finite callables; no widget, panel, or telemetry owner
is exposed to this controller. Qt views poll immutable copies of state.
"""
from __future__ import annotations
from copy import deepcopy
import threading
import time
from Workstation.workstation_core.target_discovery import (
    CANDIDATE_MAX_AGE_S, TargetReferenceSettings, discover_targets,
)


class TargetDiscoveryController:
    def __init__(self, *, connection_snapshot, connect, stop, read_preferences,
                 save_preferences, endpoint_usage=lambda: {}, discover=discover_targets):
        self._connection_snapshot = connection_snapshot
        self._connect = connect
        self._stop = stop
        self._read_preferences = read_preferences
        self._save_preferences = save_preferences
        self._endpoint_usage = endpoint_usage
        self._discover = discover
        self._lock = threading.RLock()
        self._cancel = threading.Event()
        self._thread = None
        self._stop_thread = None
        self._generation = 0
        self._closed = False
        self._result = {'state':'IDLE','generation':0,'candidates':[],'errors':[], 'overflow':0}
        self._operation = ''
        self._operation_error = ''

    def _busy(self):
        return bool((self._thread and self._thread.is_alive()) or (self._stop_thread and self._stop_thread.is_alive()))

    def _connection_active(self):
        value = self._connection_snapshot()
        return bool(value.get('worker_alive') or value.get('phase') in ('STARTING','RUNNING'))

    def discover(self, kind, preferences):
        settings = TargetReferenceSettings.from_mapping({**preferences, 'endpoint':'', 'transport':kind})
        if settings.mode != 'drone':
            raise ValueError('UAM reference is deferred; choose Drone for discovery')
        if kind not in ('usb','network'):
            raise ValueError('Unknown Target discovery kind')
        with self._lock:
            if self._closed or self._busy() or self._connection_active():
                raise RuntimeError('Stop the current Target operation before discovery')
            usage = self._endpoint_usage()
            if usage.get('discovery_busy'):
                raise RuntimeError('Another device discovery is active; retry after it finishes')
            self._generation += 1
            generation = self._generation
            self._cancel = threading.Event()
            self._result = {'state':'SEARCHING','generation':generation,'candidates':[],'errors':[], 'overflow':0}
            self._operation_error = ''
            self._thread = threading.Thread(target=self._run, args=(generation, kind, settings.to_mapping(),
                self._cancel, tuple(usage.get('endpoints', ()))), name='target-discovery', daemon=True)
            try:
                self._thread.start()
            except Exception as exc:
                self._result.update(state='FAILED', errors=[str(exc)])
                raise
        return self.snapshot()

    def _run(self, generation, kind, settings, cancel, blocked):
        try:
            result = self._discover(kind, settings, cancel, blocked=blocked)
        except Exception as exc:
            result = {'state':'FAILED','candidates':[],'errors':[str(exc)], 'overflow':0}
        with self._lock:
            if generation != self._generation or self._closed:
                return
            if cancel.is_set():
                result = {'state':'CANCELLED','candidates':[],'errors':[], 'overflow':0}
            self._result = {**result,'generation':generation}

    def cancel(self):
        with self._lock:
            if self._thread and self._thread.is_alive():
                self._cancel.set()
                self._result['state'] = 'CANCELLING'
                self._result['candidates'] = []
        return self.snapshot()

    def use_candidate(self, candidate_id, generation):
        with self._lock:
            if self._busy() or self._closed or self._connection_active():
                raise RuntimeError('Stop the current Target operation before selecting a candidate')
            if generation != self._generation:
                raise ValueError('Candidate belongs to an old search; search again')
            candidate = next((row for row in self._result['candidates'] if row['candidate_id']==candidate_id), None)
            if candidate is None:
                raise ValueError('Target candidate is no longer available')
            if time.monotonic()-candidate['observed_monotonic_s'] > CANDIDATE_MAX_AGE_S:
                raise ValueError('Candidate expired; search again')
            if candidate['identity_state'] != 'OBSERVED':
                raise ValueError(candidate['identity_state'])
            return deepcopy(candidate)

    def connect(self, payload):
        data = dict(payload)
        settings = TargetReferenceSettings.from_mapping(data)
        with self._lock:
            if self._closed or self._busy() or self._connection_active():
                raise RuntimeError('Stop the current Target operation before connecting')
            usage = self._endpoint_usage()
            if usage.get('discovery_busy'):
                raise RuntimeError('Another device discovery is active')
            if settings.mode == 'drone' and not settings.endpoint:
                raise ValueError('Enter a Target endpoint or select a discovered candidate')
            if settings.mode == 'drone':
                from Workstation.workstation_core.target_discovery import endpoint_parts
                import os
                kind, host, port = endpoint_parts(settings.endpoint)
                for item in usage.get('endpoints', ()):
                    try:
                        other, oh, op = endpoint_parts(item['endpoint'])
                    except (KeyError, ValueError):
                        continue
                    if ((kind == other == 'udp' and port == op) or
                        (kind == other == 'serial' and os.path.realpath(host)==os.path.realpath(oh))):
                        raise RuntimeError(f"Endpoint in use by {item.get('owner', 'another connection')}")
            peer = None
            if data.get('candidate_id'):
                row = self.use_candidate(data['candidate_id'], data.get('discovery_generation'))
                for key in ('endpoint','baudrate','expected_system_id','expected_component_id'):
                    if getattr(settings, key) != row[key]:
                        raise ValueError('Candidate fields changed; reselect it or use manual input')
                peer = row['peer']
            request = settings.to_mapping()
            if settings.mode == 'uam':
                request['endpoint'] = ''
            request['expected_peer'] = peer
            self._operation_error = ''
            result = self._connect(request)
            self._result.update(state='IDLE', candidates=[])
            return result

    def stop(self):
        with self._lock:
            if self._closed or self._busy():
                raise RuntimeError('A Target operation is already in progress')
            self._operation = 'STOPPING'
            self._operation_error = ''
            self._stop_thread = threading.Thread(target=self._run_stop, name='target-stop', daemon=True)
            try:
                self._stop_thread.start()
            except Exception as exc:
                self._operation = ''
                self._operation_error = str(exc)
                raise
        return self.snapshot()

    def _run_stop(self):
        try:
            self._stop(timeout_s=3.0)
        except Exception as exc:
            with self._lock:
                self._operation_error = str(exc)
        finally:
            with self._lock:
                self._operation = ''

    def save(self, payload):
        preferences = TargetReferenceSettings.from_mapping(payload)
        with self._lock:
            if self._closed or self._busy() or self._connection_active():
                raise RuntimeError('Stop Target operations before saving connection preferences')
            self._save_preferences(preferences)
        return preferences.to_mapping()

    def snapshot(self):
        with self._lock:
            return {**deepcopy(self._result), 'busy':self._busy(),
                    'operation':self._operation,'operation_error':self._operation_error,
                    'preferences':self._read_preferences().to_mapping()}

    def resource_inventory(self):
        return {'target_discovery_thread':self._thread, 'target_stop_thread':self._stop_thread}

    def shutdown(self, timeout_s=1.0):
        with self._lock:
            self._closed = True
            self._cancel.set()
            self._generation += 1
            threads = (self._thread, self._stop_thread)
        deadline = time.monotonic()+timeout_s
        for thread in threads:
            if thread is not None:
                thread.join(max(0, deadline-time.monotonic()))
        return not self._busy()
