# SASL/NGII NTRIP authentication without GGA — 2026-08-26

## Scope and privacy

- Operator entered the session password in the Workstation UI; the password and Authorization
  value were not printed, captured or stored in this evidence.
- Profile: `RTS2.ngii.go.kr:2101`, mountpoint `VRS-RTCM32`, username `SASL`, GGA policy
  `LEFT_GNSS`, no TLS.
- MAVLink and cameras were deliberately disconnected in this app run, so this gate proves caster
  authentication separately from rover GGA and RTCM injection.

## Observed boundary

- TCP state: established from the Workstation process to `210.117.198.83:2101`.
- Kernel TCP counters after connection: 153 bytes sent/acked and 14 bytes received.
- UI caster response: `ICY 200 OK`; connection `CONNECTED / VALID`.
- UI health: `STALE / INVALID`, not authentication error/401.
- RTCM frames/chunks: zero.
- GGA: `left GNSS has no message (policy=LEFT_GNSS)`; GGA sent count zero.

## Interpretation

The SASL/NGII credential was accepted; otherwise the worker would have closed with the observed
`401 Unauthorized` failure class instead of retaining an established transport. The caster sent
only its success response and withheld correction payload while the configured `LEFT_GNSS` GGA
source was unavailable. This is authentication PASS and correction-stream NOT_RUN, not a full NTRIP
or RTCM-injection PASS.

The next integrated run must use the updated P1/P2 MAVLink subscriber, obtain a fresh Left GNSS
sample, send periodic GGA, receive CRC-valid RTCM3, and prove per-side injection counters.

## Updated-app integrated follow-up

After restarting into the changed Workstation, both P1/P2 MAVLink links entered `QUALIFYING` with
the correct sysids and fresh ATTITUDE. The same session credential was applied to the exact
SASL/NGII account and NTRIP was started again.

Observed UI/runtime evidence:

- Caster response: `ICY 200 OK`; credential source `session`; reconnects 0.
- GGA: periodic send active, last send about 4.5 seconds at the captured snapshot.
- RTCM: 80 CRC-valid frames at 5.10 Hz, age 0.52 seconds.
- Transport: 25 chunks / 3,795 bytes at about 243 B/s at the snapshot.
- RTCM message types included 1006, 1008, 1013, 1030, 1031, 1032, 1033, 1074,
  1084, 1094, 1124 and 1230.
- P1 distribution showed 68 frames/68 packets/3,996 bytes and failure 0; the inspector showed
  current Left and Right delivery ages, proving both routes were active.
- Overall state: `READY`, NTRIP `RTCM_ACTIVE / VALID`, status strip `NTRIP RTCM OK`.
- Kernel TCP counters during the live stream: 772 bytes sent/acked and 10,418 bytes received.

Screenshot artifact (temporary host path): `/tmp/workstation_new_ntrip.png`, SHA-256
`19de03ee6b0af7373cd9ff211ec34058bff2e070fdf9dd6e961446648ce3824c`.

This is a real short integrated PASS for authentication, GGA, RTCM parsing and dual-route injection.
It does not prove RTK fix/convergence: the attached receivers still showed `NO_FIX` in the indoor
setup, and sustained/cold-boot/full-load gates remain.
