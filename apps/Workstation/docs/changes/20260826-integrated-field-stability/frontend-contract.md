# Frontend Contract

## Controls and inputs

Use existing Preview, MAVLink Connect, NTRIP Start, time-sync/STUS, Measurement and recording controls.
Current camera/Pixhawk endpoints and SASL/NGII profile remain operator-visible. Secrets remain masked.

## Payloads and commands

Existing typed settings/controller payloads remain authoritative. UI intent is not success; every
control waits for the backend/external postcondition defined in `backend-contract.md`.

## Visible states and transitions

- Camera shows requested/effective decoder and per-side current frame or exact error/recovery state.
- Pixhawk separates CONNECTING, QUALIFYING, LIVE, ATTITUDE STALE and ETH MAVLINK RX STALLED.
- NTRIP separates DNS/TCP, HTTP_AUTH_FAILED, CONNECTED_NO_DATA, STREAMING and route/injection state.
- STUS shows ACK/reconnect/warmup and live pair delta/SR-PC/SR age/clock quality.
- Measurement/recording shows IDLE/QUEUED/STARTING/RUNNING/STOPPING/ERROR with factual storage detail.

## Errors and recovery actions

Messages name the failed side, boundary, last valid age/result and next safe operator action. They do
not imply device reboot or success from a host reconnect. Physical power or USB action is requested
from the user only when evidence requires it.

## Keyboard, focus, accessibility, and layout

Rapid numeric/status updates use stable formats and reserved geometry. They must not move adjacent
controls, steal keyboard focus, reset selection or jump Diagnostics/Data scroll. Text conveys state
without color-only meaning; long detail remains available through existing tooltip/detail surfaces.

## Cross-feature visible behavior and isolation

One side or one writer failing changes only its dependents. Camera remains usable during NTRIP
failure; healthy P1/P2 and unrelated storage streams continue where safe. Measurement admission is
not blocked by old Session/spool or best-effort storage state.
