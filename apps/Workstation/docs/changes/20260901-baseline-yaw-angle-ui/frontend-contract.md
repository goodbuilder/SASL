# Frontend Contract

## Controls and inputs

No controls. Inputs are the selected P1→P2 NED baseline and independently available P1/P2 PX4
ATTITUDE yaw samples.

## Payloads and commands

None. The workspace reads `EvidenceAdapter.baseline_yaw_alignment(now_s=...)`.

## Visible states and transitions

- `CURRENT`: fresh horizontal baseline plus fresh P1 and P2 yaw.
- `PARTIAL`: current baseline and exactly one current yaw; the other row is STALE or WAITING.
- `STALE`: finite last values exist but no complete current comparison remains.
- `WAITING`: no finite non-zero horizontal baseline direction or no comparable yaw.

Visible fields are baseline azimuth, P1 yaw/relative angle, P2 yaw/relative angle,
P2-minus-P1 yaw, selected receiver sources/fix states, GNSS ages/pair delta, and ATTITUDE ages.

## Errors and recovery actions

Missing/non-finite/zero-horizontal inputs fail closed to `--`/WAITING with a factual reason.
Stale arrows are dashed/muted. Recovery occurs automatically when fresh source evidence returns;
no local retry is offered.

## Keyboard, focus, accessibility, and layout

Read-only values remain selectable/inspectable. Status meaning is available in text and tooltips;
plot series use labels and line style as well as color. Fixed extents prevent live-layout motion.

## Cross-feature visible behavior and isolation

Raw attitude and baseline groups remain unchanged. One missing Pixhawk affects only its own angle;
camera, Base GNSS/NTRIP, Alignment R,t, Measurement, recording, and Diagnostics remain usable.
