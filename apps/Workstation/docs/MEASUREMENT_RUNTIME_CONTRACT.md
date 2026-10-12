# Current Measurement runtime contract

This is the current contract for Run ownership, method execution and explicit recovery.
Historical Change Log entries describe earlier behavior. The recording schema owns field-level
coordinates; this document changes no geometry, units, timestamps or saved schema.

## Run ownership

`workstation_core/measurement_run.py` is the Qt-free owner of state, generation, frozen metadata,
source membership and last-run identity. The UI adapter publishes compatibility projections;
modifying those projections cannot change lifecycle decisions. Camera owners publish source facts.

Start reserves `STARTING` before optional recording callbacks, rejects duplicate/reentrant Start,
and moves to `RUNNING`. Stop releases admission to `IDLE` before optional persistence and teardown.
Old-generation completion cannot retire a new Run. Storage failure, missing telemetry or unverified
MAG selection do not gate observation. MAG state is recorded as advisory evidence.

## Frozen evidence

Pair/request/outcome/bundle nested mappings and arrays have immutable ownership. Public exports
return detached JSON objects/arrays. External aliases cannot change source evidence or hashes.
Canonical JSON sorting/separators, numeric values and wire schemas remain compatible. Replay still
uses the recorded inputs, not newer settings or sensors.

## Method execution

The registry contains ten method IDs: `pixhawk_rtk_live`, `pixhawk_rp_gps_baseline_yaw`,
`checkerboard_stereo`, `horizon_hybrid`, `mountain_gnss`, `drone_rtk_rotation_hybrid`,
`drone_rtk_multiview`, `terrain_pnp`, `combined_hybrid`, `cloud_gnss`.

Each pair submits only ready methods; unready workbook rows say not attempted and record why.
Workers belong to the application lifetime. The primary worker starts first; a terminal primary
outcome releases comparison startup. Each method has a 20-pair pending limit and child library
thread limits remain one. A missing start acknowledgement has a 30-second guard. A started
calculation has **no automatic compute deadline**; legacy TIMEOUT records remain readable.
Normal Measurement Stop and subsequent Start do not cancel earlier calculations.

## Explicit recovery

Data > Status > `Method 계산 상태` shows selected-method state, elapsed time and pending work.
Elapsed time does not prove the worker is dead. Run/Pair identity is available in the tooltip.
`선택 계산 중단·재시작` sends the displayed method, worker generation, pair and request generation.
The actor checks that identity again, preserves a result already received, confirms termination
before replacing the worker, and records an interrupted calculation as FAILED/operator_restart.
Queued work and other workers remain active. Unconfirmed termination never starts a duplicate
worker. Replacement launch failure is method-local and supports explicit retry.

Acceptance means queued intent. RESTARTED requires a replacement process to have started; it does
not imply that the next calculation has succeeded. REJECTED/FAILED remain visible. Recovery events
and terminal outcomes keep the original Run identity even if a newer Run is active. Recording is
best effort under the existing storage policy; enqueue rejection remains visible in diagnostics.

## Developer verification

From the repository root in the supported environment:

```bash
python3 tools/check_portable_paths.py
python3 -m pytest apps/StereoWIRIS/tests/test_coordinate_contract.py apps/StereoWIRIS/tests/test_stereo_geometry_common.py apps/StereoWIRIS/tests/test_triangulation.py apps/Workstation/tests
```

CI uses this full Workstation directory. Historical path exemptions added for the RCA are bound
to seven exact content hashes; editing those files or adding an adjacent file cannot inherit an
exemption. No historical evidence is rewritten. The correction plan and fresh verification are in
[the RCA packet](changes/20261009-rca-reliability-remediation/verification.md).
