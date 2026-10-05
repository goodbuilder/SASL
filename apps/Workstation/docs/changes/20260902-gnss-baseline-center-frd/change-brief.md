# Change Brief

## Operator outcome

When current P1, P2, and Drone GNSS positions are available, `Devices > Telemetry` shows one
read-only `GNSS BASELINE_CENTER F/R/D/Rg` result. The operator can see the Drone position relative
to the midpoint of the P1/P2 GNSS antennas as Forward, Right, Down, and Euclidean range in metres,
together with freshness, three-source time skew, and fix evidence.

## Motivation

The current Telemetry page already derives the scalar range from the P1/P2 midpoint to Drone GNSS
for ideal-disparity reference, but discards the directional components. Displaying the components
helps field alignment and relative-position diagnosis without requiring camera images or an
operator-entered distance.

## Included scope

- A pure GNSS-derived local-level coordinate transform with explicit frame, origin, axes, units,
  and sign conventions.
- Read-only `CURRENT`, `STALE`, and `WAITING` state with 2.0 s maximum age and operator-selected 1000 ms
  maximum three-source timestamp spread.
- A compact fixed-height one-row engineering table directly below Telemetry `Live position`.
- Reuse of the exact derived `Rg` by the existing ideal-disparity reference so the page cannot show
  two different midpoint-to-Drone distances.
- Focused pure, controller-snapshot, UI, timing/failure, and adjacent Telemetry regression tests.
- README and affected Workstation product-document updates after approval.

## Excluded scope and non-goals

- No change to the canonical camera/measurement `baseline_center_frd` output.
- No camera optical pose, P1/P2 attitude, alignment profile, antenna-to-camera lever arm, or target
  bearing/pose inference.
- No Measurement admission, recording/session schema, CSV/parquet field, settings, command,
  persistence, navigation, device, MAVLink, RTCM, worker, or topology change.
- No RTK-Fixed requirement; fix types remain visible evidence rather than a display gate.

## Acceptance summary

- For a hand-built baseline pointing East, the GNSS-derived Right axis points East, Forward points
  North, and Down follows NED Down; distinct sentinel components and `Rg` match an independent
  calculation.
- Missing/invalid geometry shows `WAITING` and never substitutes zero. Valid but old, future, or
  over-1000-ms-spread positions retain their finite last values as `STALE`, never `CURRENT`.
- P1/P2 side reversal changes the signed axes rather than silently preserving their old signs.
- The label explicitly says GNSS-derived and does not use the canonical measurement frame name.
- Live updates do not resize the group/table, move controls, steal focus, reset scroll, or add
  steady CPU/GPU work beyond constant-time arithmetic.
