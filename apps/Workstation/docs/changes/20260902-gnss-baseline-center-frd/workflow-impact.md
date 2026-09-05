# Workflow Impact

## Affected operators and workflow IDs

- `WF-TELEMETRY-GNSS-REFERENCE`: measurement operator/researcher observes relative Drone geometry
  while P1/P2/Drone GNSS is live.
- No command workflow or device-configuration permission changes.

## Current workflow

1. Connect/select P1, P2, and Drone telemetry through existing controls.
2. Inspect three WGS84 positions, fix, satellites, and age.
3. Read only scalar midpoint-to-Drone range in the ideal-disparity group.
4. Mentally infer direction elsewhere or use a separate analysis path.

## Proposed workflow

1. Existing connection and position steps remain unchanged.
2. A fixed read-only table automatically shows state, F, R, D, Rg, maximum age, maximum three-way
   timestamp delta, and P1/P2/Drone fix labels.
3. The adjacent static contract identifies GNSS antenna midpoint, local-level axes, and the fact
   that this is not camera `baseline_center_frd`.
4. Existing RTCM and ideal-disparity evidence continue below it; ideal disparity consumes the same
   `Rg` and freshness decision.

## Preserved, added, removed, and reordered steps

- Preserved: all connection, Apply/Save, RTCM, position-table, focal-length, and disparity steps.
- Added: passive visual inspection only.
- Removed: none.
- Reordered: none; the new group appears between `Live position` and `RTCM delivery` because its
  three source rows are the direct inputs.

## Decisions, confirmations, handoffs, waits, and artifacts

- No click, confirmation, payload, worker handoff, or artifact is added.
- The existing status refresh is the only trigger. UI projection is not success evidence beyond
  the source timestamps and fix data carried in the same snapshot.

## Interruption, cancellation, retry, restart, and resume

- Not applicable as a command lifecycle. Missing data produces `WAITING`; new data recomputes
  automatically. Restart begins `WAITING` until live sources return.

## Degraded, error, and recovery behavior

- Missing/invalid source or degenerate horizontal baseline: `WAITING`, F/R/D/Rg shown as `--`.
- Valid geometry with age >2.0 s, future receive time, or timestamp spread >1000 ms: `STALE`, finite
  last values remain visible with actual age/skew and reason.
- Low GNSS fix: geometry remains available when position/timing is valid; fix labels expose the
  limitation without inventing RTK readiness.
- A source recovery automatically returns the row to `CURRENT`; unrelated connection/RTCM UI
  remains usable if geometry is unavailable.

## Completion and next actions

`CURRENT` means only that the three GNSS-derived local-level values are finite, fresh, and within
the timestamp-spread limit. It does not mean camera measurement, optical alignment, RTK readiness,
or field qualification succeeded. The operator may continue existing Telemetry/Measurement work.

## Training, operating-procedure, and compatibility impact

Operators learn one sign convention: with P2 physically to the right of P1, Right is P1->P2,
Forward is Right x NED-Down, and positive Down follows decreasing altitude. Swapping P1/P2 reverses
the signed horizontal axes. Existing files/settings and procedures require no migration.

## Workflow-to-screen, command, state, and regression traceability

| Workflow behavior | Screen/read model | Regression evidence |
|---|---|---|
| Current three-GNSS result | Telemetry fixed table / snapshot mapping | pure sentinel + controller + UI tests |
| Missing or degenerate | same | WAITING boundary tests |
| Old/future/skewed | same | STALE timing tests |
| Low fix remains evidence | source/fix column | no-RTK-gate test |
| Existing disparity/RTCM preserved | current groups | `test_workstation_telemetry.py` regression |
| No geometry/focus movement | Telemetry workspace | offscreen state sequence at target sizes |
