# Change Brief

## Operator outcome

Open an irreplaceable observation interval immediately, even with no camera, telemetry, storage or
ready geometry. Sources join independently when they become available; missing sources affect only
their evidence and method results.

## Motivation

Post-capture integrity can be reviewed later. Losing the observation because an earlier artifact,
recorder, storage writer or device is unready is the larger operational risk.

## Included scope

- Immediate sessionless run admission and source-level state.
- Manual in-run recovery for cameras, P1/P2, Base GNSS/NTRIP and Target.
- New MP4 segment on camera rejoin; no path reopening.
- Bounded retry of essential structured records and non-gating artifact evidence.
- Existing compact status surfaces and OS folder/player handoff.

## Excluded scope and non-goals

No Session/package, artifact catalog, revalidation, manifest/hash/full-decode gate, internal player,
automatic failed-camera reconnect, algorithm change or coordinate conversion.

## Acceptance summary

Run state becomes RUNNING at command acceptance, source failures never roll it back, late callbacks
remain generation-bound, and a second run starts while old media/storage evidence finishes.
