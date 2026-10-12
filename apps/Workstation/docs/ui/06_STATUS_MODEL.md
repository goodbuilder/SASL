# StereoWIRIS Status Model

- Status: Revised after audit; source-backed implementation present
- Depends on: 00–05 design documents

## 1. Fundamental Rule

> Command is intent. Status is evidence.

Measurement Run state is owned by the Qt-free `MeasurementRunService`; the UI reads its projection.
`Data > Status > Method 계산 상태` exposes cached method execution state, elapsed time, pending
work and original Run/Pair. Elapsed time alone does not establish failure. A recovery request is
pending until the actor reports RESTARTED, REJECTED or FAILED. RESTARTED means process replacement,
not successful completion of a subsequent calculation. See the
[current Measurement contract](../MEASUREMENT_RUNTIME_CONTRACT.md).

## 2. Status Dimensions

Potential independent dimensions:
- Connectivity
- Activity
- Availability
- Freshness
- Validity
- Quality
- Fault
- Transition

Do not collapse everything to OK/ERROR.

## 3. Connectivity

Candidates: DISCONNECTED, CONNECTING, CONNECTED, DISCONNECTING, ERROR, UNKNOWN.

CONNECTED does not necessarily imply live valid data.

## 4. Activity

Examples:
- Camera stream: STOPPED / STARTING / LIVE / STOPPING / ERROR
- Detection: DISABLED / STARTING / RUNNING / STOPPING / ERROR
- Recording: IDLE / STARTING / ACTIVE / STOPPING / SAVED / ERROR

Actual states must be verified against code.

## 5. Availability / Freshness / Validity

Availability: AVAILABLE / UNAVAILABLE.
Freshness: FRESH / AGING / STALE / UNKNOWN.
Validity: VALID / INVALID / UNKNOWN.

Do not confuse them.

## 6. Timestamp Semantics

Freshness requires a known timestamp source: source, receive, processing, or display timestamp. UI repaint rate is not freshness.

## 7. Missing vs Zero

Use `--`, `NO DATA`, `N/A`, `STALE`, `INVALID` according to semantics. Never silently use zero for absent data.

## 8. Last Known Value

If retained after staleness, it must be visibly marked stale and expose age where useful.

## 9. UNKNOWN / DEGRADED / ERROR

UNKNOWN means insufficient evidence. DEGRADED requires a defined rule/reason. ERROR requires factual evidence.

## 10. Camera Status

Dimensions: connection, RTSP availability, requested decoder mode, actual per-side decoder,
fallback reason, frame reception, frame age, FPS, transport input metrics, and preview. Aggregate
LIVE only if actual frames are confirmed. Requested NVIDIA/Auto is not GPU success evidence.

- `GPU · nvh264dec`: actual worker reported NVDEC.
- `CPU · avdec_h264 (selected)`: explicit Software mode, not degraded.
- `CPU fallback · avdec_h264`: Auto failed NVIDIA and completed the bounded software restart; WARN.
- `Decoder mismatch`: actual decoder contradicts requested mode without fallback evidence; ERROR.
- `--`: no active decoder evidence.
- `INPUT UNVERIFIED`: a current decoded frame is live but compressed-source RTP/IDR integrity is
  not proven. Camera state remains LIVE with WARN. This provenance does not block Preview, YOLO,
  native evidence, or Measurement and cannot be promoted to source-integrity green.

Camera workspace owns the primary per-side value, compact CAM tooltip owns short actual/fallback
detail, and Diagnostics Performance owns GPU model/driver/capability detail plus the worker's
RTP/H.264 input metrics. RTP packet rate is packets per second; missing packets are estimated from
forward RTP sequence gaps; loss percentage is missing/(received+missing); H.264 AU rate is parser
access-unit output rate. These measurements are observational and do not change stream policy.
GPU utilization is diagnostic only and cannot turn a source LIVE.

## Pixhawk Radio transport status

- P1, P2 and active Drone are independent status owners with expected sysids 1, 2 and 3.
- Telemetry Drone network discovery has an independent transient state:
  `IDLE/SEARCHING/FOUND/EMPTY/AMBIGUOUS/FAILED`. A `DRONE VERIFIED` row requires a decoded sysid-3
  HEARTBEAT; it is candidate evidence only and never promotes connection state.
- `RADIO QUALIFYING 00/10s` and `RADIO LIVE` describe transport evidence only; neither implies
  `RTK READY` or Measurement admission.
- Optional Radio demand is `CRITICAL`, `OPTIONAL_25`, `OPTIONAL_50` or `STOCK`. Recovery starts
  after 30 healthy seconds and advances at 15-second intervals; a critical regression returns to
  `CRITICAL` immediately.
- `RADIO_STATUS` RSSI/noise/txbuf/error fields are diagnostics. Missing `RADIO_STATUS` is
  `UNKNOWN`, not a transport failure. Freshness, RTCM queue/age/drop and I/O evidence own
  degradation.
- A 10-second expected-heartbeat timeout, sustained all-RX loss or Radio device removal retires
  the connection generation and returns to `DISCONNECTED · PRESS CONNECT`.
- Discovery status is revisioned command evidence: `SEARCHING USB`, `SEARCHING NETWORK`,
  `FOUND n USB ENDPOINTS`, `FOUND n NETWORK CHOICES`, or `ERROR · reason`. Rows always belong to
  that latest accepted command. Empty/error results do not retain stale candidates, while saved
  assignments and connection state remain independent.
- A USB row without heartbeat is `identity unverified`; observed sysid 1/2/3 maps to P1/P2/Drone.
  A Network row with several roles is explicitly labelled `shared NIC` and never implies a shared
  Telemetry Radio.

## 11. Pixhawk Status

The subscriber owns an immutable per-side MAVLink health snapshot; widgets do not infer health from
`socket exists`, ping, or one historical heartbeat. Workstation does not expose a ROS2 transport;
Legacy transport semantics remain in the Legacy UI.

The Pixhawk workspace separately projects network assignment evidence: `LEGACY AUTO ROUTE`,
`ASSIGNED`, `REASSIGN REQUIRED`, `LINK DOWN`, `ADDRESS MISSING`, `NO ROUTE`, `ROUTE MISMATCH`,
`ENDPOINT INVALID`, and `PORT IN USE`. These states describe the host-side prerequisite and never
promote a side to CONNECTING/LIVE. Same-NIC topology is not a warning/status dimension.

Explicit role application separately uses `REVIEW REQUIRED`, `AUTHORIZING`, `STOPPING AFFECTED`,
`APPLYING`, `VERIFYING TARGET`, `CLEANING SOURCE`, `RECONNECTING`, `APPLIED`, `ROLLING BACK`,
`ROLLED BACK`, `CANCELLED`, `AUTH DENIED`, `READBACK MISMATCH`, and `ROLLBACK PARTIAL`. `APPLIED`
requires NetworkManager+kernel readback and, for a previously active role, a new Camera frame or
expected-system Pixhawk HEARTBEAT+ATTITUDE. A partial state is never promoted to assignment success.

- `DISCONNECTED`: local listener absent.
- `CONNECTING`: listener open but expected P1/P2 system identity not observed.
- `QUALIFYING`: expected identity plus fresh heartbeat/ATTITUDE, but less than 60 continuous seconds.
- `LIVE`: 60 continuous seconds complete and current HB/ATT/ANY evidence remains fresh.
- `ATTITUDE STALE`: ATTITUDE older than 2 seconds while heartbeat/other traffic remains current.
- `ETH MAVLINK RX STALLED`: all expected-system RX silent for 10 seconds; RTCM/commands blocked;
  detail may say PX4/NuttX UDP task deadlock suspected, never proven.
- `RECOVERING`: expected packet returned after stall/reboot; qualification restarted at zero.
- `PARAM SYNC`: an explicit operator/reset/apply/save readback owns a bounded receive-path
  reservation; it suppresses false stall and requires fresh post-command HB+ATT before
  qualification restarts. It is never entered automatically because a side reaches `LIVE`.
- `SHELL SYNC`: bounded Stable-PX4 `sensor_mag`/active-device Shell readback; it is not a
  parameter lookup and returns to the normal transport state after the readback.

Display endpoint, observed/expected sysid, HB/ATT/ANY ages, qualification `mm:ss / 01:00`, and RTCM
block evidence without resizing cards or moving controls. Only `LIVE` satisfies Pixhawk readiness.
This device-connection state is separate from measurement-method readiness: the live
`Pixhawk attitude + GNSS/RTK` method reuses the last valid process-local attitude and GPS sample per
side, so it can stay ready while a side's connection card is no longer `LIVE`. The device card keeps
reporting its own authoritative live state and is never rewritten by that reuse.
Ten-minute comparison and 60-minute soak durations remain separate stability evidence.

The P1/P2 `IMU stream 1/2/3` rows use actual `SCALED_IMU/2/3` app-receive timestamps. `WAITING`
means no sample has arrived; `LIVE` requires age at or below one second; `STALE` retains the last
values and reports their real age; `PARTIAL` is reserved for a fresh packet without all three
xmag/ymag/zmag values. Command send alone never changes a row to `LIVE`.

## 12. GNSS / RTK

Preserve source semantics: NO DATA, NO FIX, FLOAT, FIX, STALE, etc. Do not substitute generic OK.
The raw selected baseline is `P1→P2` in local NED `[North, East, Down]` metres.
The separate `P1 body FRD projection` state is `WAITING` until P1 attitude is available;
when ready it is `[Forward, Right, Down]` in P1 PX4 body FRD and retains its attitude age.
It must not overwrite raw NED, camera optical RDF, or public `baseline_center_frd` values.

The GNSS midpoint read model (now displayed in Baseline Yaw90 reference information) is a separate coordinate/status domain.
`CURRENT` means three finite selected GNSS positions, application-receive age <=2.0 s per source,
maximum receive-timestamp spread <=1000 ms, and a finite non-zero horizontal P1→P2 direction.
`STALE` retains the last finite signed geometry when age is old/future or spread exceeds the bound;
`WAITING` means missing/invalid geometry and uses `--`, never zero substitution. Fix labels are
evidence and do not gate state. Its frame is `gnss_baseline_center_local_level_frd`, origin is the
P1/P2 GNSS antenna midpoint, and it must not be promoted to camera/Measurement
`baseline_center_frd`. Existing ideal disparity uses the same Rg/readiness.

The same GNSS read model also exposes the acute horizontal installation angle
`atan2(|Forward|,|Right|)` between the P1→P2 baseline line and the midpoint→Drone GNSS vector,
plus `90° error=|90°−angle|`. `CURRENT`, `STALE`, and `WAITING` follow the existing three-GNSS
age/timestamp-spread/finite-geometry state. Drone ATTITUDE/yaw is not an input, and the 90° error
is an operator measurement rather than a GOOD/WARN/BAD acceptance verdict.

The Pixhawk `Dr` geometry projection is a separate latest-state aid. It displays midpoint→Dr in
`gnss_baseline_center_local_level_frd`, P1→Dr in P1 `px4_frd`, and P2→Dr in P2 `px4_frd`, each
with `F/R/D/Rg` metre values and source age. `∠Dr-P1-P2` and `∠Dr-P2-P1` are independent
unsigned internal angles calculated from horizontal GNSS North/East vectors only; Down and yaw
are excluded. Missing/non-finite rays are `WAITING`, stale finite source geometry is `STALE`,
and no new global status key, command, persistence, or measurement semantic is introduced.

Pixhawk magnetometer rows expose raw and calibration-corrected yaw without changing the global
status model. `RAW yaw°` is `wrap180(atan2(-y,x))` in the producer sensor axes and is diagnostic
only. `Corrected yaw°` follows PX4's `Magnetometer::Correct()` order (`R * S * (data - offset)`) using
the device's read-back `CAL_MAG` slot; it remains waiting until those parameters are manually read.
The IMU table contains the three standard `SCALED_IMU/2/3` rows and the dedicated RM3100
`DEBUG_VECT` `CAN1_MAG`/`CAN2_MAG` rows; RM3100 rows retain the `mavlink/DEBUG_VECT` topic and
Gauss unit.
On Stable PX4, the same RM3100 rows may instead be populated by the bounded Shell
`listener sensor_mag` path and retain a `px4-shell/sensor_mag/<instance>` topic/source.
`PX4 yaw°` is visible in the operator table when the row's device identity exactly matches the
active `vehicle_magnetometer` and ATTITUDE is fresh; it is the firmware estimator yaw with the
configured `EKF2_MAG_DECL` declination handling. Hidden `Δyaw°` remains the backend diagnostic
difference `PX4−RAW`. `WAITING`, `STALE`, `PARAM 대기`, `PARAM 불완전`, and `전류 보정 대기`
remain visible as corrected-yaw states. The Pixhawk MAG control adds side-specific `PARAM APPLYING`,
`PARAM VERIFIED`, `VERIFYING`, `ACTIVE VERIFIED`, `STALE`, `TIMEOUT`, and `ERROR` evidence; it
does not create a global status key. Live-Pixhawk Measurement admission consumes the public
side-specific verification snapshot and records its selection provenance; no stream request,
firmware change, or persistent parameter schema is added.

## 13. Detection

Baseline Yaw90 keeps each live row independent. The azimuth uses the selected P1/P2 GNSS now,
not the optional Drone/attitude aggregate; its source freshness follows the existing baseline
read model. P1/P2 Roll/Pitch retain raw body-FRD attitude and real ages. Vertex angles retain
the three-GNSS timing contract. Producer frozen-position evidence is displayed as FROZEN without
adding a new detector or changing measurement admission. Installation snapshots never select or
apply a Measurement method. Reverse is writable while idle (or inactive ERROR), and locked during
queued/start/running/stop or a pending camera/Detection start; this is separate from DOF fill.
Inspector details selected from Baseline Yaw90 carry a selection-owner tag. Leaving that Method
clears only those details and restores normal context projection; it never keeps a stopped
installation snapshot labelled CURRENT and never polls the hidden setup pane to fake freshness.
Display query exceptions and malformed snapshot containers use local `QUERY_ERROR` (shown as
`조회 오류`). Values come from an immutable last-successful display snapshot, with elapsed
monotonic time added to the recorded ages. The table and Inspector resolve the same display rows,
including explicit missing rows. Successful reads restore the source's CURRENT/STALE/WAITING
state; no prior values are fabricated when there has been no observation. Query failure is not a
Pixhawk disconnect or a Measurement readiness decision. Last-valid Measurement attitude/GNSS caches
are never mutated by this fallback; current calculation inputs cannot be inferred from old UI data.

Separate configured/model-loaded/runtime-running/target-present/output-fresh states where applicable.

## 14. Measurement

Candidate states: UNAVAILABLE, WAITING, VALID, DEGRADED, REJECTED, STALE, ERROR. These are hypotheses until mapped to existing measurement logic.

Rejection reasons must come from existing logic, not the UI.

Measurement result validity and input-media provenance are independent dimensions. Existing gates
may produce `VALID` while the same row/status also says `INPUT UNVERIFIED`; the UI must show both
without relabelling the result DEGRADED or hiding the provenance.

Operating K,D readiness is likewise separate from optical-state provenance. Missing K,D, invalid
immutable profile identity, and unknown/mismatched native left/right frame resolution are blocking.
Zoom readback presence/age/value, left/right zoom agreement, camera serial, and stabilization state
remain saved evidence only; they never create a blocker/advisory or change method readiness.

## 15. Recording

Manual recording: IDLE → STARTING → ACTIVE → STOPPING → IDLE, with DEGRADED/ERROR diagnostics.
Measurement run enters RUNNING immediately after its in-memory generation/time boundary is frozen.
Source states (`PENDING/WAITING_RESOURCE/LIVE/UNAVAILABLE/LEFT`) are independent and never own run
admission.

Decoded-NV12 NVENC recording independently reports queue drops and writer details. The compact
post-run state is presence-only: `SAVED`, `PARTIAL`, or `FAILED`; its detail explicitly says playback
integrity is not verified. Recorder failure, missing files, artifact-write failure or oldest-drop
never changes Detection or Measurement state.

The Recording workspace also exposes an advisory protocol projection (not a new
runtime status key). It evaluates camera LIVE, RTCP/NTP SYNCED, detection RUNNING,
active calibration, and the selected Pixhawk/GNSS or checkerboard geometry evidence.
It returns `READY` only when all required checks pass; it never treats a command
acceptance as proof that a source is live. Storage state is observable but never a Measurement gate.

The Data Status stream projection exposes the authoritative JSONL path/state and the independent
same-stem CSV path/state. CSV `FAILED`/`RETRYING`/`DROPPED` is diagnostic evidence only; JSONL and
Measurement remain available and `Retry Storage` is the recovery action.

### 15.1 Measurement XLSX

Data shows `IDLE/WRITING/READY/FAILED` from the workbook writer, plus `REBUILDING` and its
separate error during saved-input reconstruction. A valid existing latest path is READY after
restart. Pending Method jobs and per-record state remain visible. Raw JSONL/CSV health is
independent of workbook publication and appears in the report diagnostics.

The report has five front tabs, including a dedicated R,t tab; Method tables also show expanded
R,t. Missing source rows remain absent and reader cells use `—` (core evidence uses `N/A`).
Long paths/errors stay in fixed-height labels/tooltips, preserving control geometry and focus.
CSV retry can restore unmirrored rows from JSONL; RTCM mirror failures are storage diagnostics,
not evidence that correction injection itself failed. Source failures never disable Measurement.

The optional bulk stream `gnss.baseline.center` follows the existing store IDLE/PENDING/OK/
DEGRADED/FAILED state and written/dropped/error counters. Live Telemetry CURRENT is not storage
success. The stream may fail/drop independently without changing GNSS reception, Measurement or
recording lifecycle. Latest XLSX Summary and `GnssBaselineCenter` expose only admitted rows from the
selected Measurement generation.

## 15.1 Camera Clock, Frame Timing, and Media

These are independent status domains. `Clock VERIFIED` requires fresh decoder-independent RTCP
Sender Reports and a robust Left/Right relative delta at or below the configured 100 ms default.
Signed/raw per-camera SR-PC remains a diagnostic series and persisted evidence; its absolute value
cannot create FAILED, revoke VERIFIED, trigger Auto, or block a result. A robust three-SR median/MAD
window tolerates one isolated SR outlier.

`Frame timing CURRENT` requires same-logical-clock-epoch frame mapping. Time-dependent pairing
accepts pairs at or below 100 ms and may drop an occasional unmatched/outlier candidate; it does not
promise one output pair per source frame. `Media` independently reports decoded/video/MP4 truth, so
Clock may remain VERIFIED while Frame timing or Media is unavailable.

Live distance timing reports separate requested and effective bases. The default request is
`APP RECEIVE`; `RTCP/NTP preferred` becomes effective only with VERIFIED/current same-epoch frame
evidence. Missing, malformed, stale, transitioning, epoch-mismatched or over-limit RTCP timing
activates exact app-receive fallback. The selected delta is at most 100 ms, both app/NTP deltas stay
visible, and fallback alone does not degrade an otherwise valid distance or turn Clock green.

Calibration Live/Auto timing also uses decoded-frame app-receive monotonic for its configurable
`Max Live pair Δt`, but it reports `HOST_RECEIVE_PROXY` rather than precision/exposure verification.
Missing/invalid receive evidence or an over-limit delta yields `INTRINSICS ONLY`; RTCP/NTP remains
separately recorded and never replaces the missing app clock. FTP keeps its `HOST_DISPATCH_PROXY`.

Calibration Auto candidacy separately reports `waiting for NAL1 non-delta` and `exact native
FrameKey expired`. These are recoverable waiting states, not Camera failures. Only a type-1,
non-delta AU whose exact PTS reaches the decoded frame can proceed; an evicted/wrong-generation key
is never replaced by a neighbor. This state does not change Preview, YOLO, Measurement, recording,
or manual capture status.

## 15.2 Automatic Optical-Pose Suggestion

Automatic suggestion quality and permission are separate dimensions. Horizon and Mountain show the
detector state (`GOOD`, `LOW_CONFIDENCE`, or `NO_DATA`) unchanged and separately project
`COPY READY` only after structural validation. Horizon requires current identity/dimensions,
exactly two finite in-frame endpoints per side and 25%-width separation. Mountain requires current
identity, paired exclusions, equal counts, minimum four and unique/distributed in-frame points.
Otherwise `COPY BLOCKED` shows the structural reason. Low-quality `AUTO_ACCEPTED` provenance
retains its original quality state and warning; acceptance never relabels it `GOOD`.

## 15.3 Calibration Capture Feedback

Calibration capture status is an additive read model owned by the Calibration controller. It keeps
three independent dimensions visible: latest per-side checkerboard detection, current-process
capture inspection outcomes, and persisted current-root inventory. A capture command or Auto timer
tick is not success. The run request count advances only after the existing checkerboard validation
worker is admitted; stereo/K,D-only/failure counts advance from its result. Root change or restart
resets only the run counters.

The persisted inventory uses the same pair-usage evidence as calculation input. Missing roots are
valid `EMPTY` evidence with zero counts; unreadable roots are `UNAVAILABLE` with `--`, never false
zero. `Stereo 저장 후보` is explicitly before manual exclusion and solver pose filtering. Preview
evidence older than the existing two-second Auto freshness limit is `STALE`; found state always
includes text/corner count and never relies on color alone. No capture-feedback state may promote
calibration quality or Apply readiness.

## 16. Aggregate Status

Aggregate status may simplify details for Navigation/Status Bar but must be computed by explicit rules.

## 17. Global Status Candidates

CAM L, CAM R, P1, P2, RTK, DET, MEAS, REC, FPS. Include only reliable, operationally useful states.

## 18. Detail Levels

Level 1: Global Status — compact summary.
Level 2: Workspace/Inspector — operational detail.
Level 3: Diagnostics — technical evidence/history.

## 19. State Ownership

Preferred flow:
Backend/Controller → Authoritative Status Model → Status Bar/Navigation/Workspace/Diagnostics.

Do not let each widget invent its own truth.

## 20. False Positive / False Negative Prevention

Prevent false success such as CONNECTED because Connect was clicked, RECORDING because button is active, VALID because a number exists, RTK FIX because GNSS exists.

Prevent false failure caused by arbitrary UI timeouts, optional messages, or valid zero values.
Do not turn USB reboot command acceptance into `RECOVERED` or `LIVE`; expected Ethernet evidence and
the full new qualification are required.

## 21. Startup / Shutdown

Use UNKNOWN vs DISCONNECTED according to actual knowledge. Expected STOPPING/DISCONNECTING/SAVING transitions are not errors.

## 22. Summary

> Display evidence, not optimism.

Implementation note: `BackendStatusAdapter` is the aggregate status projection and
`EvidenceAdapter` supplies detailed read-only values. A command being accepted does not
make a device live, a recording saved, or a measurement valid; the corresponding source
status and age/clock evidence must confirm it.

## Target reference status additions

Search state is independent: IDLE/SEARCHING/CANCELLING/FOUND/EMPTY/PARTIAL/CANCELLED/FAILED. Device presence without a heartbeat is NO HEARTBEAT; unsupported zero IDs are not verified. Candidate observation is distinct from a connected target.

The existing Target controller phase/stream_state remain authoritative for connection. Async Stop has a separate STOPPING operation. HEARTBEAT/GPS_RAW_INT/GPS2_RAW/ATTITUDE use host-monotonic receive age: UNKNOWN without evidence, STALE after3s, otherwise VALID/INVALID according to the packet fields and GPS fix. RTK_FIXED/RTK_FLOAT describe the reported fix alongside age. No target status is promoted to calibration truth or a new Measurement admission gate.
