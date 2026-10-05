# MAVLink Connect and PARAM-reservation evidence — 2026-08-27

## Artifacts

| Artifact | SHA-256 | Purpose |
|---|---|---|
| `/tmp/mavlink_postfix_connect_20260827.pcap` | `67063d19ccc6e9688d181dc0bf4034dd2fb61e4b27c29553966ad084810aa127` | latest-app Connect/profile gate |
| `/tmp/workstation_postfix_connect_5s.png` | `ff735ca8be268c72dce953bdcd57f09d00ad619d35831dfd2ee8083d97af797c` | responsive post-Connect UI |
| `/tmp/mavlink_postfix_telemetry_10m_20260827.pcap` | `5ae330dfd274aa3d811b406fb298aecc99b4d4c5395755f4488caf24daf419bc` | ten-minute telemetry and automatic PARAM refresh |
| `/tmp/workstation_postfix_telemetry_10m.png` | `db35a59d00858fcb66de7f5fd1d06e1b16f852558d7ef80d0250691d26ff036b` | LIVE followed by PARAM SYNC |
| `/tmp/workstation_postfix_after_param_sync.png` | `6c520e4e4a2758ae0775b8210f8dd3b73e7a0ddf2cb0c0650f4d43297142addd` | post-command requalification state |
| `/tmp/camera_only_mav_stall_20260827.pcap` | `a65f0374ff2d0040726ed83063508b1e10bf33b2f6b4ee38787307c6b385c5c5` | separate real inbound-wire stall |

Raw pcaps remain outside the repository because they are runtime evidence. Paths and hashes are
recorded here so later qualification can distinguish the exact artifacts.

## Connect result

The 69.63-second latest-app Connect capture proves that the GUI-freeze fix reached its external
postconditions: the UI remained responsive, both input queues drained, expected P1/P2 traffic
continued, and each side received exactly one 15-command `MAV_CMD_SET_MESSAGE_INTERVAL` Stock
profile. This supersedes the earlier “post-fix real Connect NOT_RUN” note, but does not close the
long-run Ethernet-stability requirement.

## PARAM reservation defect

The ten-minute telemetry capture reached LIVE before the one-shot automatic calibration refresh.
The refresh then exposed three deterministic host defects:

- Each side sent 142 `PARAM_REQUEST_READ` packets but returned only 124 unique parameters.
- The 18 unanswered names were exactly `CAL_MAG4_ID`, `CAL_MAG4_PRIO`, rotation/orientation, and
  the twelve `CAL_MAG4_*` correction values. Every one introduced an approximately 1.5-second gap;
  their sum accounts for about 27 seconds of the refresh.
- PX4 exposes four Stock calibration instances (`CAL_MAG0_*` through `CAL_MAG3_*`). The local PX4
  source independently declares `Magnetometer::MAX_SENSOR_COUNT = 4` and `MAX_MAGS = 4`.
- While the per-side command worker owned `recv_match` and the side send lock, PX4 inbound traffic
  remained continuous (ATTITUDE maximum gap about 0.038/0.039 seconds and remote heartbeat about
  1.005/1.004 seconds), but the host GCS heartbeat stopped for 28.055616 seconds on P1 and
  27.318256 seconds on P2.
- When the historical FIFO was replayed, each old `HEARTBEAT -> ATTITUDE` pair ran current-time
  stale-ATTITUDE recovery. That emitted 28 redundant message-id-30 interval requests on P1 and 27
  on P2.

The implemented software contract is therefore:

1. Stock PX4 calibration refresh reads slots 0–3 only: 75 magnetometer plus 49 attitude parameters,
   124 total. The generic explicit parameter API remains unrestricted.
2. A PARAM/shell command retains exclusive receive ownership and the side send lock. Its worker
   services GCS heartbeat from the same re-entrant lock owner at one-second cadence; the GUI path
   uses a nonblocking lock attempt.
3. RTCM and other writers remain excluded for the command duration, and the healthy peer continues
   through its independent receive/send owner.
4. Deferred telemetry retains ingress liveness timestamps and FIFO downstream delivery. Historical
   HEARTBEAT replay cannot emit setup, Stock-profile, or stale-ATTITUDE recovery actions. The next
   genuinely live HEARTBEAT owns any warranted action.
5. The deliberate post-command qualification reset remains unchanged.

## Residual distinction

The later camera-only capture is a different failure: for 9.08 seconds the host continued sending
P1/P2 heartbeat (10/9 packets) while inbound wire traffic was zero. PARAM refresh was not the cause
of that interval. It confirms a real Ethernet MAVLink receive stall, but not the PX4 internal
IOB/sendto mechanism; onboard task/IOB/backtrace evidence is still required for that root cause.
