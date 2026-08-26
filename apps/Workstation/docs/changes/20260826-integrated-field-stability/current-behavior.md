# Current Behavior

## Version and worktree context

- Historical implementation baseline: `c4397995d864576c966702cfd3c398b6032f8990`.
- The accumulated approved source was committed as `1198689fafed1ef392d395e11437352431312ac8`;
  the initial integrated packet update is `981aad0292a0a1a03974f6637105b4a38ab7929b`.
  At packet audit time the branch matched origin. Existing crash logs and failed-venv backup remain
  untracked and user-owned.
- Existing relevant packets are approved/verifying independently: sessionless Measurement,
  canonical runtime, NVDEC status, Ethernet MAVLink, blank Legacy startup/manual intrinsics, and
  WIRIS preview/frame decoupling. This packet owns their integrated qualification and only new
  defects discovered by that qualification.
- No Workstation/StereoWIRIS/ffmpeg/GStreamer runtime owned device resources when this packet was
  initialized; UDP `14550/14551` were free.

## Reproduction

- Cameras are isolated on `enx588694fce07e` at `10.0.0.11/32` with preferred-source `.231/.232`
  host routes. P1/P2 are on replacement RTL8156B `enx0023a40a2e39` at `10.0.0.10/24` with
  explicit `.2/.3` routes; neither profile supplies a default route.
- Left/Right camera (`10.0.0.231/.232`) and P1/P2 (`10.0.0.2/.3`) answer ping on their owning NIC.
- Both RTSP TCP `8554` and NGII `2101` accepted TCP connections.
- Fresh ffprobe connections to both `/visible` streams reported repeated `non-existing PPS 0`,
  `decode_slice_header error`, `no frame`, and initially `0x0` dimensions.
- A subsequent 12-second GStreamer `nvh264dec` probe produced 1280x720 CUDA-memory output and RTCP
  Sender Reports, but that was decoder output rather than an independently decodable boundary. The
  current NAL5+exact-PTS fail-close rejects both FW 2.18.72 zero-IDR senders, so usable video remains
  FIELD FAIL rather than presenting concealed/corrupt frames as healthy.
- NTRIP authentication, RTCM bytes, MAVLink heartbeat/ATTITUDE, STUS and recording were not started
  during the initial setup check.
- A later read-only passive diagnostic bound each MAVLink endpoint for six seconds. Both P1 and P2
  timed out with no heartbeat while ping remained healthy; no ROS2/uXRCE samples were present.
- USB topology exposed two Auterion PX4 FMU v6X CDC devices with the same serial `0`; by-id is
  therefore ambiguous. Heartbeat mapped `/dev/ttyACM1` to P1/sysid 1 and `/dev/ttyACM0` to
  P2/sysid 2. Both report PX4 `1.17.0`.
- Read-only parameter requests proved P1/P2 `MAV_2_CONFIG=1000`, `MODE=0`, `BROADCAST=1`,
  rate 2,000,000 and correct 14550/14551 remote/local ports. `ifconfig` proved link UP and
  `10.0.0.2/.3` with DR `10.0.0.10`.
- USB `mavlink status` showed both Ethernet instances present with broadcast enabled but
  `tx=0.0 B/s` and increasing `txerr` (about 39 B/s P1 and 52 B/s P2).
- A no-setting-change A/B bound the host's expected port and sent GCS 255/190 heartbeat at 1 Hz.
  P1 immediately returned 613 datagrams/25,261 bytes in 12 seconds; P2 returned 596/24,748.
  This proves pre-identity GCS heartbeat/known-peer seeding is required on the current firmware.
- Secure credential storage is available but has no saved SASL/VRS-RTCM32 password, and the named
  legacy username/password environment variables are absent. Live NTRIP authentication therefore
  requires an operator-entered session password after every app restart. The latest session entry
  passed isolated Stop/reconnect and a 636.5-second correction/injection gate; the password remains
  absent from settings/keyring.

## Observed frontend behavior

Startup correctly reports missing active intrinsics rather than auto-applying a stale profile. The
shutdown warning was traced to a disconnect worker stopping the GUI-owned MAVLink receive timer;
full disconnect now quiesces it on the GUI owner. A real two-run zero-IDR failed-start sequence used
distinct run IDs and rolled back cleanly; final Quit returned zero with empty stderr and no process,
socket, thread or shared-memory residue. A later active-Measurement `SIGKILL` also released its
camera children, established sockets and owner-named native rings. On restart the app did not adopt
the incomplete run, presented `MEAS WAITING / REC IDLE` with enabled `측정 시작`, and created a
distinct run ID. Startup additionally reclaims only same-UID Workstation native rings whose embedded
owner PID is dead; a controlled tracker-less orphan passed that fallback boundary.

A later latest-source launch reproduced a stronger Pixhawk Connect freeze. The worker opened both
UDP sockets and sent one initial GCS heartbeat per side, but the UI stayed `DISCONNECTED`, stopped
accepting input, and both Linux Recv-Q values reached 213,760. SIGUSR1 placed the GUI thread in
`_handle_mavlink_status_changed -> _refresh_session_toggle_buttons -> _set_button_role ->
QStyle.polish`; completion therefore never started the subscriber receive timer. Wire traffic stayed
CRC-valid and gap-free at about 66.9/66.1 unique messages/s per side. NTRIP and Preview were off.

The host also had an independently runaway Orca process (about 64% CPU, 3.4 GiB RSS and 2.9 GiB
swap) and a saturated AT-SPI destination queue. This is a strong external amplifier, not native-
stack proof that Orca alone caused `polish()` not to return. Accessibility remains enabled; the
product fix removes synchronous style repolish from the completion critical path.

The subsequent latest-app Connect gate remained responsive and sent the exact 15-command Stock
profile once per side. A following telemetry-only run reached dual LIVE for ten minutes, then the
automatic calibration refresh exposed a separate command-reservation defect. The old app requested 142
parameters per side although only 124 existed; all 18 missing responses were nonexistent
`CAL_MAG4_*` names and accounted for about 27 seconds of timeout. During that interval inbound PX4
heartbeat/ATTITUDE remained continuous, but the command-held send lock starved host GCS heartbeat
for 28.055616 seconds on P1 and 27.318256 seconds on P2. Historical heartbeat/attitude replay then
emitted 28/27 redundant ATTITUDE interval commands. The raw artifacts, hashes, boundary distinction
and software contract are recorded in
`evidence/mavlink-param-reservation-20260827.md`. After the fix, the first isolated adapter returned
124/124 parameters per side in 0.227/0.250 seconds with host heartbeat gaps below 1.045 seconds while
camera Preview ran. The replacement RTL8156B separately passed dual-RTSP load for 45 seconds and the
latest-app ten-minute high-rate/PARAM gate with zero wire, NIC or app error. These isolation results
remove the former immediate camera-load trigger but do not prove the PX4-internal #25956 mechanism.

## Observed backend and persistence behavior

Persisted Workstation settings select TCP/100 ms/Auto, MAVLink UDP `14550/14551`, and network RTK direct via
`RTS2.ngii.go.kr:2101/VRS-RTCM32` with `LEFT_GNSS` GGA. Current applied sync limits are pair delta
50 ms, SR-PC offset 100 ms, clock-rate error 0.5%, two SRs, 30-second warmup, and SR age 7 seconds.
The visible `SR stale=7.0 s` value was Applied and saved before the real closed-loop attempts.

The pre-slice STUS worker dispatched Left and Right concurrently and treated their responses only
after both requests had already been sent. The verification timer stopped at the first VERIFIED
result, while `rtcp_sr_age_s` remained the age captured in the last frame metadata. Therefore a
Left failure could still mutate Right, and two frozen streams could preserve a stale green state.
Focused RED tests reproduced both defects before the ordered command and continuous-health changes.

The first ordered real STUS still failed the hard quality gate. Exact pcap reconstruction found
successful HIWS RTT medians of about 515 ms Left and 543 ms Right, all discarded by the 250 ms cap,
so the initial target lead was effectively zero. Retained unique pad-probe SRs show signed
`SR NTP - PC receive UTC` medians of -736.135 ms Left and -1,307.044 ms Right. The displayed
+916.9/+1,427.6 ms values were mixed frame receive delays, not the actuator signal. The bounded
closed-loop slice and its independent oracle are recorded in
`evidence/stus-closed-loop-correction-20260827.md`. The 15-second hardware A/B later returned exact
Left OK in 3.691 seconds followed by Right OK in 3.433 seconds. Post-seed SR-PC medians were
-313.712/-808.643 ms, leaving about 494.931 ms relative offset. Both replacement generations then
emitted zero NAL5, so the current-frame gate blocked round two; R3 remains CRITICAL FIELD FAIL.

## Documentation, code, test, and runtime conflicts

- The latest complete Workstation suite passes 1466 tests with 1 skip, but required target-hardware
  gates remain open or failed; software PASS cannot close this packet.
- Port-open/ping evidence conflicts with prior protocol silence and therefore is only a prerequisite.
- Pre-identity heartbeat/peer seeding, the Connect receive-drain freeze and the replacement-adapter
  application ten-minute gate are resolved. Remaining R4 work is cold-boot/final 60-minute
  qualification.
- Root pytest import-name collision and a StereoWIRIS native Qt order crash remain visible. The
  repository portability audit now passes; same-host fresh checkout/venv launch checks pass, while a
  separate clean Ubuntu OS/VM remains unverified.
- Workstation startup/Auto/Enable/Reactivate hardware evidence proves zero automatic encoder SET.
  The operator later approved only `SEIR VISIBLE 0 -> 1`; exact live readback is now `0/1/30/1` on
  both cameras. Sequential reboots proved the IDR value is not persistent; restoring/reapplying it
  during active RTSP still produced zero NAL5 on both FW 2.18.72 devices. Both finish restored at
  `GETH=TRUE`, `0/1/30/1`. Stored JSON remains non-oracle, Workswell firmware/vendor owns the sender
  fix. Legacy startup/receive/Preview/restart/record pcap also captured zero TCP/2240 packets, so the
  pre/post no-mutation gate passes. Its blank Record action now starts both camera recorders rather
  than finalizing an empty no-MP4 session; the produced MP4 remains integrity-failed on zero IDR.
