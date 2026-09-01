# Change Brief

## Operator outcome

On `Devices > Pixhawk`, directly below the existing `P1→P2 baseline` group, the operator can
read the live horizontal angle from each P1/P2 Pixhawk Forward direction to the selected
P1→P2 GNSS baseline and verify the underlying source/freshness evidence.

## Motivation

The current workspace shows P1/P2 yaw and NED/FRD baseline values in separate regions. The
operator must calculate their horizontal relationship mentally, while the older owner-only panel
contains a different unsigned 3D Forward-to-baseline aid that is not exposed in the Workstation.

## Included scope

- Signed horizontal P1 and P2 baseline-relative angles, baseline azimuth, and P2-minus-P1 yaw.
- Source-backed CURRENT/PARTIAL/STALE/WAITING presentation with ages and selected GNSS sources.
- A compact fixed-height north-referenced top view using existing theme tokens.
- Pure geometry tests, evidence/read-model tests, Qt integration tests, and operator documentation.

## Excluded scope and non-goals

- No target angle, tolerance, GOOD/WARN/BAD alignment verdict, history, or stability statistic.
- No command, worker, device/protocol, measurement algorithm, setting, or navigation change.
- No new persisted derived-angle field or recording schema change.
- No replacement of raw NED, P1 body FRD, camera RDF, or public measurement coordinates.

## Acceptance summary

Fresh finite baseline and attitude inputs produce independently sourced P1/P2 signed angles at the
next normal UI poll. Missing or stale sources remain explicit, rapid updates do not move controls
or scroll, and existing raw telemetry/recording behavior is unchanged.
