# Combined Preview/NTRIP P2 MAVLink stall — 2026-08-26

## Preconditions

- Updated Workstation, P1/P2 Ethernet connected and initially `QUALIFYING` with correct sysids.
- SASL/NGII NTRIP `ICY 200 OK`, periodic GGA and CRC-valid RTCM around 4.7-5.1 Hz.
- RTCM was routed to P1/P2 with failure count zero.
- Left/Right RTSP Preview used TCP/100 ms and Auto selected real `nvh264dec` on both sides.
- No device setting, firmware, parameter, reboot or encoder command was sent by Codex.

## Initial combined success

Both camera panes reached LIVE from current frames. UI showed Left/Right `GPU · nvh264dec`, frame age
about 0.09-0.10 seconds and decoded rates about 16.5/12.4 Hz. Parent RSS was about 431 MiB; camera
children used about 315/313 MiB RSS and 102 MiB GPU memory each. NTRIP/RTCM and both Pixhawks were
active concurrently.

Screenshot: `/tmp/workstation_preview_live.png`, SHA-256
`9cd9b7d19af3c280281529b7729863222fc58b3b360bd58073896ee6ec759ca3`.

## Failure

Within the short combined window, P2 transitioned to `ETH MAVLINK RX STALLED` while P1 remained
`QUALIFYING`. RTCM became stale for the affected path. Host socket evidence showed P1 Recv-Q 2560
and P2 Recv-Q 0.

A five-second packet capture on `enx588694fce07e` recorded 599 packets with kernel drop 0. It showed
continuous `10.0.0.2:14550 -> 10.0.0.10:14550`, host heartbeat to both peers, and no inbound P2 UDP.
Therefore the failure was not a GUI/socket-drain backlog; P2 stopped emitting on the wire.

P2 USB heartbeat also timed out while ping to `10.0.0.3` remained 3/3. This broadened the observed
failure from Ethernet receive alone to the P2 MAVLink service layer.

## Physical recovery

The user physically reapplied power to P2. During boot the device appeared as PX4 bootloader and
P2 ping temporarily failed. After firmware boot, `10.0.0.3` returned 3/3 ping and the app's existing
UDP socket received sysid 2 heartbeat/ATTITUDE without reconnecting the host listener.

The UI entered `RECOVERING` with P2 HB about 0.51 s, ATT/ANY about 0.01 s. P1 also entered
RECOVERING after the physical/network interruption, and both began a new 10-minute qualification.

Screenshot: `/tmp/workstation_p2_recovery.png`, SHA-256
`f48c338618e42ee964d811a208cc36af9456c705bb65ecf9c7a47c0381937adb`.

## Verdict

- PASS: app detects real P2 wire silence, fail-closes the correction path, preserves P1, and accepts
  recovered heartbeat on the existing socket after named physical recovery.
- FAIL: P2 still stalls under combined workload; the pre-identity startup fix does not solve this
  separate device/service failure.
- OPEN: complete recovery qualification, one-side isolation repetition and root-cause A/B for
  RTCM load versus camera/network load.

## Recovery did not sustain while Preview remained active

The first Stop Preview UI attempt did not take effect; both RTSP child processes remained alive.
After P2 power recovery, P1/P2 initially emitted fresh ATTITUDE and entered RECOVERING. With Preview
still active, both later stopped emitting on the wire. The UI showed P1/P2 RX_STALLED with ages
about 235/241 seconds, while the host continued its one-second GCS heartbeat to each peer.

Screenshot: `/tmp/workstation_both_stalled.png`, SHA-256
`d327079830a800d966cc16045759d758e6a3ba0d04cd4a4433d2d17a16cb4205`.

This makes the combined-load result a dual-link FIELD FAIL, not a completed recovery qualification.
Preview was subsequently stopped and both RTSP child processes/sockets were confirmed absent. A new
P1+P2 physical recovery and MAVLink-only qualification is required before load-isolation A/B.
