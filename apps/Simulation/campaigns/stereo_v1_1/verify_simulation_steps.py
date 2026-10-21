#!/usr/bin/env python3
"""Ground-only Gazebo pause/step/resume proof; never arms or sends MAVLink.

ACK only proves request acceptance. Each 5/5/1-step request must independently
produce observed source-clock increments 20/20/4 ms with unchanged epoch/local
reset generation. This advances real simulation physics; it never resets time.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from campaigns.stereo_v1_1.ros_observation_bridge import RosObservationBridge
from campaigns.stereo_v1_1.runtime_observation import ObservationCache
from campaigns.stereo_v1_1.simulation_stepper import SimulationStepper, wait_clock_evidence


def run_probe(stepper, get_snapshot, read_clock, *, timeout_s=2.0, records=None):
    """Injectable observation boundary; all admitted stamps originate in ROS."""
    records = [] if records is None else records
    def wait_snapshot(predicate, label):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            snap = get_snapshot()
            if snap is not None and predicate(snap):
                return snap
            time.sleep(.002)
        raise RuntimeError(f'simulation step control: probe timeout: {label}')

    initial = wait_snapshot(lambda snap: True, 'fresh initial snapshot')
    generation = (initial.simulation_epoch, initial.xy_reset_counter, initial.z_reset_counter)
    def guard(snap):
        if (snap.simulation_epoch, snap.xy_reset_counter, snap.z_reset_counter) != generation:
            raise RuntimeError('simulation step control: probe epoch/local reset changed')
        return snap

    stepper.warmup_continuous()
    warmed = wait_clock_evidence(read_clock, generation[0], generation[1:],
                                baseline_ns=initial.simulation_time_ns, quiescent=False,
                                timeout_s=timeout_s)
    records.append({'action': 'service_warmup_observed', 'clock_evidence': warmed})
    stepper.pause()
    quiet = wait_clock_evidence(read_clock, generation[0], generation[1:],
                               baseline_ns=warmed['source_timestamp_ns'], quiescent=True,
                               timeout_s=timeout_s)
    records.append({'action': 'clock_quiescence_observed', 'clock_evidence': quiet})
    paused = get_snapshot()
    if paused is None:
        raise RuntimeError('simulation step control: probe clock paused but coherent snapshot unavailable')
    guard(paused)
    observed_ns = paused.simulation_time_ns
    records.append({'action': 'pause_observed', 'source_timestamp_ns': observed_ns,
                    'host_monotonic_ns': time.monotonic_ns(), 'quiescence_s': .1})
    for steps in (5, 5, 1):
        target_ns = observed_ns + steps * stepper.physics_dt_ns
        requested_ns = time.monotonic_ns()
        stepper.step(steps)
        ack_ns = time.monotonic_ns()
        def arrived(snap):
            guard(snap)
            if snap.simulation_time_ns > target_ns:
                raise RuntimeError('simulation step control: probe source clock overshot step target')
            if snap.simulation_time_ns < observed_ns:
                raise RuntimeError('simulation step control: probe source clock moved backwards')
            return snap.simulation_time_ns == target_ns
        actual = wait_snapshot(arrived, f'{steps} physics steps')
        records.append({'action': 'step_observed', 'physics_steps': steps,
                        'previous_source_timestamp_ns': observed_ns,
                        'source_timestamp_ns': actual.simulation_time_ns,
                        'observed_delta_ns': actual.simulation_time_ns-observed_ns,
                        'requested_monotonic_ns': requested_ns, 'ack_monotonic_ns': ack_ns,
                        'observed_monotonic_ns': time.monotonic_ns()})
        observed_ns = actual.simulation_time_ns
    stepper.resume()
    resumed = wait_snapshot(lambda snap: guard(snap).simulation_time_ns > observed_ns,
                            'continuous clock resumed')
    records.append({'action': 'resume_observed', 'source_timestamp_ns': resumed.simulation_time_ns,
                    'host_monotonic_ns': time.monotonic_ns()})
    return {'status': 'PASS', 'schema_version': 'simulation_step_probe_v1',
            'method': 'ground_only_observed_pause_step_resume',
            'no_arming_or_mavlink': True, 'ack_is_not_clock_evidence': True,
            'physics_dt_ns': stepper.physics_dt_ns, 'simulation_epoch': generation[0],
            'xy_reset_counter': generation[1], 'z_reset_counter': generation[2],
            'records': records}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world-name', default='stereo_ground_drone')
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    if args.report.exists():
        parser.error('report already exists; refusing to overwrite evidence')
    if not args.report.parent.is_dir():
        parser.error('report parent directory must exist')
    bridge = None
    stepper = None
    result = {'status': 'FAIL', 'schema_version': 'simulation_step_probe_v1',
              'world_name': args.world_name, 'no_arming_or_mavlink': True}
    try:
        cache = ObservationCache()
        bridge = RosObservationBridge(cache)
        stepper = SimulationStepper(args.world_name)
        def observe():
            if bridge.last_error:
                raise RuntimeError(f'simulation step control: probe observation failed: {bridge.last_error}')
            admission = {}
            snapshot = cache.snapshot(time.monotonic_ns(), diagnostic_details=admission)
            result['latest_snapshot_admission'] = admission
            return snapshot
        result.update(run_probe(stepper, observe, lambda: cache.clock_observation(time.monotonic_ns()),
                                timeout_s=5.0, records=result.setdefault('records', [])))
    except BaseException as exc:
        result.update(status='FAIL', failure_type=type(exc).__name__, failure_reason=str(exc))
    finally:
        if stepper is not None:
            try:
                stepper.resume()
            except Exception as exc:
                result.update(status='FAIL', cleanup_resume_error=str(exc))
        if bridge is not None:
            try:
                bridge.close()
                result['observer_shutdown_verified'] = bridge.diagnostic_shutdown_complete
                if not bridge.diagnostic_shutdown_complete:
                    result.update(status='FAIL', cleanup_observer_error='observer did not stop')
            except Exception as exc:
                result.update(status='FAIL', cleanup_observer_error=str(exc))
        with args.report.open('x', encoding='utf-8') as handle:
            json.dump(result, handle, indent=2, allow_nan=False)
            handle.write('\n')
            handle.flush()
            os.fsync(handle.fileno())
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
