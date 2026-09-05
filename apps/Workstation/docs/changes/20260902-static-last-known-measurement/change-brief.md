# Change Brief

## Operator outcome

The operator can deliberately measure a physically static setup after live P1/P2
Pixhawk attitude or GPS delivery stops, using the last complete valid sensor set
instead of seeing only `준비된 측정 방식 없음`.

## Motivation

Field hardware may remain mechanically fixed while one telemetry path disconnects.
The current live method rejects attitude received before the current Measurement and
rejects a missing P1/P2 GPS pair even when previously observed values remain available.

## Included scope

- `Pixhawk attitude + GPS/RTK` readiness and measurement input selection.
- A clear operator-visible distinction between live and retained static inputs.
- Age/source/provenance evidence for every retained attitude and GPS sample.
- Automatic fallback on disconnect and automatic recovery when newer valid live input arrives.

## Excluded scope and non-goals

- No interpolation or prediction of vehicle motion.
- No fabrication of a missing side that has never produced a valid sample.
- No coordinate-frame or triangulation formula change.
- No change to camera decoding, detection, recording, GNSS correction, or other
  optical-pose measurement methods.

## Acceptance summary

Automatic fallback on disconnect or stale reception, reset-on-restart, no manual clear action,
no persistent retained-static warning/age display, and no new retained marker/log event are
accepted. Existing exact input/timestamp preservation is an unresolved hard boundary.
