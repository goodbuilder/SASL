# Frontend Contract

## Controls and inputs

No new controls or editable inputs. Existing Magn tables consume the same evidence refresh and
existing row-click Inspector action.

## Payloads and commands

No new operator payload. Existing Pixhawk connection lifecycle starts a read-only Shell observation
using `listener sensor_mag` and `listener vehicle_magnetometer`; the table receives additive
read-only fields from the evidence adapter.

## Visible states and transitions

Each row has:

- `RAW yaw`: finite raw sensor-axis azimuth when x/y are usable; otherwise `--`.
- `PX4 yaw (selected)`: current fresh ATTITUDE yaw only when the row's device ID exactly matches
  the active selected device; otherwise `미선택`, `identity 확인 대기`, or `--`.
- `Δyaw`: wrapped `PX4 yaw - RAW yaw` only when both values are valid.

The table preserves current LIVE/PARTIAL/STALE/WAITING/DISCONNECTED states. No state is upgraded
because a yaw calculation succeeded.

## Errors and recovery actions

No modal errors. Invalid/nonfinite/near-vertical vectors, missing identity, missing ATTITUDE, and
stale evidence show bounded text and preserve the Inspector contract. Recovery is the existing
fresh telemetry, active-device verification, or PARAM refresh action.

## Keyboard, focus, accessibility, and layout

Cells remain non-editable and selectable. Existing row selection and horizontal scrolling remain
usable; fixed formatting prevents width jitter. Text states do not rely on color alone.

## Cross-feature visible behavior and isolation

The output is a read-only Pixhawk diagnostic aid. It does not modify Alignment, Measurement,
Recording, GNSS, or camera state, and those workflows remain usable if yaw evidence is unavailable.
