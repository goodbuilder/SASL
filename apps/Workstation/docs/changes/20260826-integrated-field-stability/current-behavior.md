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
50 ms, SR-PC diagnostic reference 100 ms, clock-rate error 0.5%, two SRs, 30-second warmup, and SR
age 7 seconds. The accepted current contract changes the relative pair default to 100 ms; an older
persisted 50 ms remains a stricter operator value until Apply. SR-PC absolute magnitude is no longer
a success/revocation/result gate. The visible `SR stale=7.0 s` value was Applied and saved before the
real closed-loop attempts.

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

The latest approved implementation supersedes that frame-coupled gate with a decoder-independent
parallel RTCP observer. STUS mutation remains ordered. Convergence now removes only the Left/Right
differential around the observed SR-PC common mode and uses a 100 ms relative bound; absolute SR-PC
remains display/evidence only. One isolated observation is tolerated by robust median/MAD, and an
occasional unmatched frame may be dropped. This software path is focused-test PASS but has not yet
completed a new hardware STUS or sustained field gate.

A subsequent mutation-free 30-minute dual RTCP holdover collected 381/377 SRs with no observer
restart and confirms that Right runs faster than Left by about 16.6 ppm in both NTP and RTP media
clock regressions. The direct relative phase changed +31.356 ms over 30 minutes. This predicts about
100 minutes for 100 ms accumulation at the observed rate, but does not establish rate stability
across reboot or temperature. See
`evidence/stus-relative-oscillator-30m-20260827.md`.

## App-receive-authoritative live distance

The approved Workstation live-distance amendment is implemented. Every supported decoded-frame
reader captures one exact host-monotonic nanosecond value at frame receipt and propagates it as
`app_receive_monotonic`. Pair selection and the hard maximum delta use only that value; the maximum
is 100 ms and a saved stricter value remains allowed. NTP magnitude, Clock failure, STUS state and
logical clock-epoch changes cannot select, pause, flush, degrade or validate a distance pair.

Accepted app-timed pairs retain normal `VALID` timing when their independent geometry and input
requirements pass. NTP/SR/RTP mappings, NTP pair delta, Clock status/reason and epoch remain stored
as record-only evidence. New schema-v4 rows make `sync_delta_ns` the selected app delta, while
schema-aware readers retain the original meaning of existing v3 data. Measurement settings expose
a fixed read-only `APP RECEIVE` basis, the app delta as authoritative, and NTP delta as record-only.

The focused amendment set passes 244 tests and the final complete Workstation suite passes 1515
tests with 1 skip. Offscreen, AT-SPI and visual UI checks pass. Sustained target-hardware validation
is still NOT_RUN because both current FW 2.18.72 camera streams emit zero NAL5, preventing any new
decoded-frame pair from reaching this boundary. This is a field-media blocker, not a software PASS
for live pairing.

## Documentation, code, test, and runtime conflicts

- The latest complete Workstation suite passes 1515 tests with 1 skip, but required target-hardware
  gates remain open or failed; software PASS cannot close this packet.
- Port-open/ping evidence conflicts with prior protocol silence and therefore is only a prerequisite.
- Pre-identity heartbeat/peer seeding, the Connect receive-drain freeze and the replacement-adapter
  application ten-minute gate are resolved. Remaining R4 work is cold-boot/final 60-minute
  qualification.
- Root pytest import-name collision and a StereoWIRIS native Qt order crash remain visible. The
  repository portability audit now passes. Same-host fresh checkout/venv launch checks and a
  separate official Ubuntu 22.04.5 KVM exact-commit recreation both pass, closing R10.
- Workstation startup/Auto/Enable/Reactivate hardware evidence proves zero automatic encoder SET.
  The operator later approved only `SEIR VISIBLE 0 -> 1`; exact live readback is now `0/1/30/1` on
  both cameras. Sequential reboots proved the IDR value is not persistent; restoring/reapplying it
  during active RTSP still produced zero NAL5 on both FW 2.18.72 devices. Both finish restored at
  `GETH=TRUE`, `0/1/30/1`. Stored JSON remains non-oracle, Workswell firmware/vendor owns the sender
  fix. Legacy startup/receive/Preview/restart/record pcap also captured zero TCP/2240 packets, so the
  pre/post no-mutation gate passes. Its blank Record action now starts both camera recorders rather
  than finalizing an empty no-MP4 session; the produced MP4 remains integrity-failed on zero IDR.

The latest indoor decoder-free A/B removes decoder choice from the zero-IDR hypothesis. Raw Annex-B
captures after exact Left `SEIR/GEIR` values 0, 1, 2 and 3 contained no NAL5; FFprobe classified the
observed I pictures as non-key NAL1. Repeated read-only SDK access again collapsed the Right camera's
FTP/SDK/RTSP services while ping stayed alive. Operator physical power recovery restored the camera,
and both devices finish with all services open at `GETH=TRUE`, `GSSS=0`, `IRCR=FALSE`, visible
`0/1/30/1`. The public SDK cannot close the sender defect.

The latest pre-NTRIP A/B also removes one physical adapter as the shared-network explanation. Both
the original adapter and the replacement RTL8156B reproduce Pixhawk ETH MAVLink loss when cameras
and P1/P2 share one Layer-2 path. On the replacement, a clean no-camera 602/613-datagram gate passed,
then dual Preview drove only P2 ETH MAVLink silent while P1 and both cameras remained current. Host
heartbeats continued, P2 inbound stayed zero, and P2 USB heartbeat remained alive. NTRIP was not
started. The isolated two-NIC topology remains required for the next valid NTRIP comparison.

## PX4 calibration PARAM automatic-readback baseline — superseded 2026-08-27

Before the approved amendment, every generic MAVLink status refresh checked whether each side had
first reached `LIVE`. The first qualifying refresh launched one `calibration_refresh` per side and
marked that side for the rest of the connection session. It read 124 named calibration parameters,
populated only the calibration/MAG diagnostic caches and recording MAG-slot evidence, and reset the
post-command qualification clock. Disconnect cleared the marker so a later connection could run it
again. No Measurement, attitude, GNSS, RTCM, or camera calculation consumed those caches as a gate.

There was no operator setting to disable the automatic request. Existing P1/P2 and Advanced manual
refresh controls already provide the same readback on demand. The approved current contract removes
only the automatic trigger; historical 600-second field captures remain evidence of the old build.

## 60-second operational MAVLink qualification — current 2026-08-27

The per-side liveness owner now requires 60 continuous seconds of fresh expected-system HEARTBEAT
and ATTITUDE for `LIVE`. The same interval applies after reconnect, confirmed remote reboot, resumed
all-RX stall and explicit/manual command completion. ATTITUDE-stale at two seconds and all expected
RX stalled at ten seconds remain unchanged and immediately remove dependent eligibility.

Reaching `LIVE` sends no PARAM request because the automatic calibration readback was removed.
Existing manual parameter paths still show `PARAM SYNC`, require fresh post-command HB+ATT and then
start a new 60-second interval. UI/status/evidence project the snapshot-owned `01:00`; the external
qualification monitor requires LIVE after one minute but may continue for ten or 60 minutes as
independent long-run evidence.

## Mountain GNSS translation gate — current 2026-08-28

Mountain no longer requires both capture sides to be `RTK_FIXED`. Translation becomes observable
when both numeric fix types are at least 3, both frozen-capture positions are finite, and their
P1-to-P2 world-NED baseline is finite and non-zero. Fix type 2, missing/malformed fix evidence,
missing/non-finite position or zero baseline still fails closed. Robust Mountain rotation gates are
unchanged and remain independent.

A non-fixed usable candidate carries its exact fix types and an explicit warning, so the frozen
`mountain_gnss` comparison is `ESTIMATED`. The next Measurement freezes the latest Draft before
Detection; global Apply is not needed for comparison evidence. `pixhawk_rtk_live` remains the
representative result and existing rows are unchanged.

## Mountain automatic-suggestion copy contract — current 2026-08-29

`Auto-select Mountain` still commits only an overlay proposal. The shared `Use suggestion`
button now separates detector quality from copy usability: `GOOD`, `LOW_CONFIDENCE`, or another
quality label does not block Mountain when current frozen identity and paired native-point
structure pass. Horizon now uses the same quality-versus-copy separation.

The shared structural validator and controller independently require matching current Left/Right
frame IDs/hashes, equal original pair counts, valid shared exclusion indices, at least four
retained finite in-frame pairs, and unique/distributed points on both sides. Acceptance applies the
same exclusions to both sides and persists original/retained counts plus a canonical filtered-input
hash. The accepted annotation therefore contains exactly the reviewed overlay points.

`Solve GNSS + Mountain R,t` is unchanged and still reads only manual/accepted annotations. Direct
suggestion use by Solve, auto-copy, auto-Solve and auto-Apply remain excluded. Focused optical-pose
tests pass 76 cases, the dependent set passes 139, and the complete Workstation suite passes
1564 with 2 skips.

For Horizon, `GOOD`/`LOW_CONFIDENCE`/`NO_DATA`, confidence, residual and detector reason remain
quality evidence. Copy readiness instead requires current frozen Left/Right identity and dimensions,
exactly two finite in-frame endpoints per side, and endpoint separation of at least 25% of native
width. The controller independently revalidates and normalizes endpoints before recording
`AUTO_ACCEPTED`; Solve and Apply remain explicit and unchanged.

## Settings-preferred distance timing baseline — 2026-08-31

Current product documents and source implement fixed app-receive pairing. Settings load/migration
normalizes legacy live timing values to app receive, the Measurement basis combo is read-only, and
both pair selection and frozen Measurement evidence use app time while NTP is record-only.

The user now requests an operator-selected preference with app receive as the default and mandatory
failure fallback. This conflicts with the approved fixed-basis product contract and therefore
requires a new amendment decision, additive requested/effective/fallback evidence and a coordinated
Settings/UI/backend change. The earlier defect in which live camera state silently forced
`camera_rtc_required` demonstrates why the coordinator and freeze boundary must share one immutable
policy rather than independently reading widgets or connection state.

## Settings-preferred distance timing implementation — current 2026-08-31

Measurement Settings now exposes `App receive (default)` and `RTCP/NTP preferred · fallback App
receive`. Missing/invalid settings use app receive and legacy `camera_rtc_required` normalizes to
the RTCP preference. A running Measurement reads its frozen request from run/package metadata; idle
Apply and each new Measurement retire prior pair candidates.

The coordinator emits one immutable timing decision. Healthy RTCP requires public Clock VERIFIED,
Frame timing current, finite latest timestamps, matching nonempty current epochs and selected delta
inside the configured/100 ms bound. Every RTCP timing fault rematches on exact app-receive ns.
Freeze, method workers, UI and schema-v5 recording consume the same requested/effective/delta/reason
decision. PX4 attitude/GNSS lookup remains host-monotonic. Fallback is not added to geometry
warnings, so it does not by itself lower otherwise-valid output below VALID.

## Mountain lever-arm baseline — current 2026-09-01

`Solve GNSS + Mountain R,t` converts the same-burst P1-to-P2 antenna baseline from world NED metres
to Left camera optical RDF `[Right, Down, Forward]`. An optional payload correction, also in Left
camera RDF metres, is added to produce the optical-center baseline. No current GUI control or
persistent setting supplies that correction. When it is absent, the controller uses exactly
`[0,0,0] m`, records `lever_arm_status=UNMODELED`, adds an `UNMODELED` candidate warning, and the
Alignment details display that status. An explicit finite three-vector records `CALIBRATED` and
changes the derived Right camera center.

The visible solver tooltip already states the zero-offset assumption (“camera optical center
equals GNSS antenna”). Therefore removing only the word `UNMODELED` would not change `R,t` or the
physical baseline; it would change provenance and warning-driven result presentation. No measured
P1/P2 mounting offsets or authoritative field calibration were found in the repository.
