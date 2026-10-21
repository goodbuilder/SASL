"""Own Gazebo pause/step/resume; ACK is never evidence physics has advanced.

Only WorldControl.pause/multi_step are written. Physics dt remains 4 ms.
The caller must wait for the actual ROS clock and reference dispatch barrier
before issuing another request. This is source pacing, not timestamp replay.
"""
from __future__ import annotations

import time



class SimulationStepper:
    physics_dt_ns = 4_000_000
    max_steps = 5

    def __init__(self, world_name, *, node=None, request_type=None, response_type=None):
        if not world_name or '/' in world_name:
            raise ValueError('simulation step control: invalid Gazebo world name')
        if node is None:
            try:
                from gz.transport13 import Node
                from gz.msgs10.world_control_pb2 import WorldControl
                from gz.msgs10.boolean_pb2 import Boolean
                node, request_type, response_type = Node(), WorldControl, Boolean
            except Exception as exc:
                raise RuntimeError(f'simulation step control: Gazebo transport unavailable: {exc}') from exc
        self.node = node
        self.request_type = request_type
        self.response_type = response_type
        self.service = f'/world/{world_name}/control'
        self.paused = False
        # Discovery is independent from command acceptance. Complete it before
        # arming; never retry a multi_step after an ambiguous ACK timeout.
        deadline = time.monotonic() + 5.0
        while not self.node.service_info(self.service):
            if time.monotonic() >= deadline:
                raise RuntimeError('simulation step control: service discovery timed out')
            time.sleep(.02)

    def _request(self, *, pause, steps=0):
        request = self.request_type()
        request.pause = pause
        if steps:
            request.multi_step = steps
        try:
            ok, response = self.node.request(self.service, request, self.request_type,
                                             self.response_type, 200)
        except Exception as exc:
            raise RuntimeError(f'simulation step control: service request failed: {exc}') from exc
        if not ok or not response.data:
            raise RuntimeError('simulation step control: service request not acknowledged')

    def warmup_continuous(self):
        """Idempotent real roundtrip before arming; discovery alone is insufficient."""
        if self.paused:
            raise RuntimeError('simulation step control: cannot warm up paused session')
        self._request(pause=False)

    def pause(self):
        # An ACK timeout does not prove pause was not applied; cleanup must resume.
        self.paused = True
        self._request(pause=True)

    def step(self, steps):
        if not self.paused or not isinstance(steps, int) or not 1 <= steps <= self.max_steps:
            raise ValueError('simulation step control: require paused world and 1..5 steps')
        self._request(pause=True, steps=steps)

    def resume(self):
        if self.paused:
            self._request(pause=False)
            self.paused = False

    @classmethod
    def next_target(cls, observed_ns, end_ns):
        if observed_ns >= end_ns:
            raise ValueError('simulation step control: no remaining science interval')
        steps = min(cls.max_steps, (end_ns-observed_ns+cls.physics_dt_ns-1)//cls.physics_dt_ns)
        return steps, observed_ns + steps*cls.physics_dt_ns


def check_step_barrier(progress, target_ns, end_ns):
    """True only for observed target plus completed reference send/end event."""
    observed = progress['observed_timestamp_ns']
    if observed is None:
        return False
    if observed > target_ns:
        raise RuntimeError('simulation step control: observed clock advanced beyond requested target')
    if observed < target_ns:
        return False
    if target_ns >= end_ns:
        return progress['finished']
    return progress['last_reference_timestamp_ns'] == target_ns


def wait_clock_evidence(read_clock, expected_epoch, expected_resets, *,
                        baseline_ns, quiescent, keepalive=lambda: None,
                        check_stop=lambda: None, timeout_s=2.0):
    """Separate actual clock receipt/progress from full coherent snapshot gates."""
    deadline = time.monotonic()+timeout_s
    last_stamp = baseline_ns
    quiet_since = None
    quiet_first_receipt_ns = None
    while time.monotonic() < deadline:
        check_stop()
        keepalive()
        evidence = read_clock()
        if evidence['simulation_epoch'] != expected_epoch:
            raise RuntimeError('simulation step control: clock epoch changed during admission')
        resets = evidence['local_reset_counters']
        if resets is not None and tuple(resets) != tuple(expected_resets):
            raise RuntimeError('simulation step control: local frame reset during admission')
        stamp = evidence['source_timestamp_ns']
        if stamp is not None and stamp < last_stamp:
            raise RuntimeError('simulation step control: clock moved backwards during admission')
        if evidence['fresh'] and stamp is not None:
            if not quiescent and stamp > baseline_ns:
                return evidence
            if quiescent:
                if quiet_since is None or stamp != last_stamp:
                    quiet_since = time.monotonic()
                    quiet_first_receipt_ns = evidence.get('receive_monotonic_ns')
                elif time.monotonic()-quiet_since >= .1:
                    received_ns = evidence.get('receive_monotonic_ns')
                    # A cached stamp held for 100 ms is not evidence of a live
                    # paused publisher. Require distinct actual callbacks through
                    # the end of this same-source-clock observation window.
                    if (received_ns is not None and quiet_first_receipt_ns is not None
                            and received_ns > quiet_first_receipt_ns
                            and received_ns >= round(quiet_since*1e9)+100_000_000):
                        return evidence
            last_stamp = stamp
        else:
            quiet_since = None
        time.sleep(.02)
    label = 'received clock did not become quiescent' if quiescent else 'continuous clock did not advance after warmup'
    raise RuntimeError(f'simulation step control: {label}')


class PauseAdmissionRecovery:
    """Bounded pre-science recovery; every retry requires a fresh full stable gate."""
    def __init__(self):
        self.attempts = 0

    def begin(self):
        self.attempts += 1
        if self.attempts > 2:
            raise RuntimeError('simulation step control: pause admission attempts exhausted')

    def recover(self, stepper, stable_samples):
        stepper.resume()
        stable_samples.clear()
        if self.attempts >= 2:
            raise RuntimeError('simulation step control: coherent snapshot unavailable after second pause admission')
