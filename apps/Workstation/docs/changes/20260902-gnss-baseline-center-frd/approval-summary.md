# Approval Summary

This document explains the proposed modification in plain language. Technical detail remains in the other packet documents.

## What changes and what stays the same

`Devices > Telemetry` gains one read-only `GNSS BASELINE_CENTER F/R/D/Rg` table. It uses the live
P1, P2 and Drone GNSS positions already shown on that page. Connection controls, RTCM, positions,
ideal disparity, Measurement, recording, calibration and hardware behavior remain available.

This is deliberately not the existing camera measurement coordinate. Its origin is the midpoint
of the P1/P2 GNSS antennas. Right is horizontal P1->P2, Down is NED Down, and Forward is
Right x Down. The screen says `GNSS-derived` and `not camera baseline_center_frd`.

## Operator workflow before and after

Today the page shows only the scalar midpoint-to-Drone range. Afterward, the operator does nothing
extra: whenever all three positions arrive, the same area automatically shows signed F/R/D and Rg,
plus maximum age, maximum three-source time difference and all three fix labels.

## Success, delay, rejection, failure, overload, and recovery

- `CURRENT`: finite geometry, every source no older than 2.0 s, and their receive times within
  1000 ms.
- `STALE`: geometry remains visible but a source is old/future or their times are too far apart.
- `WAITING`: a position is absent/invalid or horizontal P1->P2 cannot define Right.
- RTK Fixed is not required; fix remains visible evidence.
- Recovery is automatic on the next good snapshot. No modal, retry button, worker or queue exists.

## Data and compatibility effects

No setting, saved file, session, CSV/parquet, Measurement result, or migration changes. The
existing ideal-disparity range uses this same Rg/readiness so the page cannot display conflicting
distances. Stale/skewed three-GNSS evidence will no longer produce a READY ideal disparity.

## Performance priorities and allowed degradation

Only constant-time arithmetic on three in-memory points is added to the existing status snapshot.
New CPU/GPU workers, I/O and queues are forbidden. If geometry is unavailable, only this table and
dependent ideal-disparity reference wait; connection, position and RTCM views continue normally.

## Visual and interaction changes, including dynamic-layout stability

One compact one-row engineering table is inserted directly below `Live position`, before RTCM.
It uses existing Qt styling and a static coordinate-contract hint. WAITING/STALE/CURRENT and long
numbers replace text in fixed cells; they do not move controls, focus, selection, scroll, docks or
the window. No dashboard cards, decorative plot, color-only state or new navigation is added.

## How acceptance will be demonstrated

- Independent NED sentinel calculations prove F/R/D signs and Rg.
- Missing, vertical/zero baseline, stale, future, 1000 ms boundary, side reversal and low-fix cases.
- Controller and real Telemetry widget tests prove the complete read-model handoff.
- Offscreen WAITING/CURRENT/STALE screenshots and geometry/focus/scroll checks at target sizes.
- Existing Telemetry/GNSS/UI tests, full pytest and `./run.sh --check` are rerun.

## Cost, risks, assumptions, rollback, and field limits

The output is an antenna-midpoint, local-level reference. It does not correct antenna-to-camera
offset, camera tilt or attitude, and it cannot validate physical P1/P2 side assignment. Swapping
P1/P2 reverses signed horizontal axes. Code rollback is additive and needs no data/device rollback.
Real hardware comparison remains separate manual evidence; fake tests are not reported as field
proof.

## Exact approved scope

Approve only the read-only GNSS-derived transform, immutable Telemetry snapshot mapping, fixed
Telemetry table/status/hint, shared Rg/readiness for ideal disparity, tests, README and listed
product docs. Exclude attitudes/optical pose/lever arms, commands, settings, persistence, recording,
Measurement gates/schemas, device/protocol changes, workers, and unrelated cleanup.

Approved by the operator on 2026-09-02 with the 2.0 s age and 1000 ms three-GNSS spread limits.
