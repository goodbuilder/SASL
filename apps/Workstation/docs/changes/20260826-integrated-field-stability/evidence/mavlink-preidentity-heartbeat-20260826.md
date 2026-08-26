# MAVLink pre-identity heartbeat evidence — 2026-08-26

## Safety and ownership

- Read-only USB requests: HEARTBEAT, AUTOPILOT_VERSION, PARAM_REQUEST_READ and `mavlink status`/`ifconfig`.
- No parameter, firmware, file, reboot or power command was issued.
- Workstation GUI was open but did not own UDP 14550/14551.
- Each live probe exclusively bound the intended host UDP port and released it afterward.

## Device identity and current configuration

| Side | USB | sysid | firmware | IP | UDP ports | rate | broadcast |
|---|---|---:|---|---|---|---:|---:|
| P1 | `/dev/ttyACM1` | 1 | 1.17.0 | 10.0.0.2 | 14550/14550 | 2,000,000 | 1 |
| P2 | `/dev/ttyACM0` | 2 | 1.17.0 | 10.0.0.3 | 14551/14551 | 2,000,000 | 1 |

Both USB devices advertise the same Auterion serial `0`, so the single `/dev/serial/by-id` symlink
is not a stable side identity. `MAV_2_CONFIG=1000`, `MAV_2_MODE=0`, link UP and DR
`10.0.0.10` were confirmed on both sides.

## Failure before the probe

A passive six-second `udpin` listener on each approved port received no heartbeat while both
Pixhawks continued to answer ping. USB `mavlink status` showed the Ethernet instances present and
broadcast enabled, but `tx=0.0 B/s`; `txerr` was increasing at about 39 B/s on P1 and 52 B/s on P2.

## Independent GCS heartbeat A/B

A minimal socket bound the same host input port and sent MAVLink GCS heartbeat at 1 Hz to the named
PX4 peer. It made no persistent change.

| Side | Duration | GCS heartbeats | Inbound datagrams | Inbound bytes | Source |
|---|---:|---:|---:|---:|---|
| P1 | 12 s | 12 | 613 | 25,261 | 10.0.0.2:14550 |
| P2 | 12 s | 12 | 596 | 24,748 | 10.0.0.3:14551 |

## Updated subscriber short gate

The changed `DualPixhawkMavlinkSubscriber` seeded each peer on its exclusive bound socket, sent
GCS source `255/190` heartbeat before inbound identity, and required an expected-sysid HEARTBEAT to
verify identity.

After 12 seconds:

- P1: `QUALIFYING`, sysid 1, 346 ATTITUDE samples, HB age 0.124 s, ATT age 0.041 s.
- P2: `QUALIFYING`, sysid 2, 354 ATTITUDE samples, HB age 0.623 s, ATT age 0.020 s.
- Both sent the exact Stock stream request profile after heartbeat identity.
- No automatic calibration PARAM refresh ran before LIVE.

## Interpretation

The current firmware/IP/parameter/link configuration was sufficient once a GCS peer was established
from the exact bound port. This proves the previous Workstation order created a startup deadlock:
the subscriber waited for inbound identity before sending the heartbeat that caused PX4 to begin
delivering to that listener. It does not yet prove 10-minute or 60-minute stability.

## Updated subscriber 10-minute qualification

The same changed subscriber then ran for 610 seconds with both sides active. It emitted one progress
sample per minute and stopped early on initial timeout, RX stall or ERROR; none occurred.

| Side | State | ATTITUDE samples | HB age | ATT age | ANY age |
|---|---|---:|---:|---:|---:|
| P1 | LIVE | 18,300 | 0.103 s | 0.020 s | 0.020 s |
| P2 | LIVE | 18,299 | 0.579 s | 0.020 s | 0.020 s |

The observed pre-change failure interval of about 5 minutes 30 seconds was crossed without a drop.
This is one warm/current-power-cycle qualification, not one of the three required independent cold
boots and not the final 60-minute combined-load soak.
