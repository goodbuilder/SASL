# StereoWIRIS Status Model

- Status: Revised after audit; source-backed implementation present
- Depends on: 00–05 design documents

## 1. Fundamental Rule

> Command is intent. Status is evidence.

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
fallback reason, frame reception, frame age, FPS, and preview. Aggregate LIVE only if actual frames
are confirmed. Requested NVIDIA/Auto is not GPU success evidence.

- `GPU · nvh264dec`: actual worker reported NVDEC.
- `CPU · avdec_h264 (selected)`: explicit Software mode, not degraded.
- `CPU fallback · avdec_h264`: Auto failed NVIDIA and completed the bounded software restart; WARN.
- `Decoder mismatch`: actual decoder contradicts requested mode without fallback evidence; ERROR.
- `--`: no active decoder evidence.

Camera workspace owns the primary per-side value, compact CAM tooltip owns short actual/fallback
detail, and Diagnostics Performance owns GPU model/driver/capability detail. GPU utilization is
diagnostic only and cannot turn a source LIVE.

## 11. Pixhawk Status

The subscriber owns an immutable per-side MAVLink health snapshot; widgets do not infer health from
`socket exists`, ping, or one historical heartbeat. Workstation does not expose a ROS2 transport;
Legacy transport semantics remain in the Legacy UI.

The Pixhawk workspace separately projects network assignment evidence: `LEGACY AUTO ROUTE`,
`ASSIGNED`, `REASSIGN REQUIRED`, `LINK DOWN`, `ADDRESS MISSING`, `NO ROUTE`, `ROUTE MISMATCH`,
`ENDPOINT INVALID`, and `PORT IN USE`. These states describe the host-side prerequisite and never
promote a side to CONNECTING/LIVE. Same-NIC topology is not a warning/status dimension.

- `DISCONNECTED`: local listener absent.
- `CONNECTING`: listener open but expected P1/P2 system identity not observed.
- `QUALIFYING`: expected identity plus fresh heartbeat/ATTITUDE, but less than 10 continuous minutes.
- `LIVE`: 10 continuous minutes complete and current HB/ATT/ANY evidence remains fresh.
- `ATTITUDE STALE`: ATTITUDE older than 2 seconds while heartbeat/other traffic remains current.
- `ETH MAVLINK RX STALLED`: all expected-system RX silent for 10 seconds; RTCM/commands blocked;
  detail may say PX4/NuttX UDP task deadlock suspected, never proven.
- `RECOVERING`: expected packet returned after stall/reboot; qualification restarted at zero.
- `PARAM SYNC`: bounded receive-path reservation; it suppresses false stall and requires fresh
  post-command HB+ATT before qualification restarts.
- `SHELL SYNC`: bounded Stable-PX4 `sensor_mag`/active-device Shell readback; it is not a
  parameter lookup and returns to the normal transport state after the readback.

Display endpoint, observed/expected sysid, HB/ATT/ANY ages, qualification `mm:ss / 10:00`, and RTCM
block evidence without resizing cards or moving controls. Only `LIVE` satisfies Pixhawk readiness.

## 12. GNSS / RTK

Preserve source semantics: NO DATA, NO FIX, FLOAT, FIX, STALE, etc. Do not substitute generic OK.
The raw selected baseline is `P1→P2` in local NED `[North, East, Down]` metres.
The separate `P1 body FRD projection` state is `WAITING` until P1 attitude is available;
when ready it is `[Forward, Right, Down]` in P1 PX4 body FRD and retains its attitude age.
It must not overwrite raw NED, camera optical RDF, or public `baseline_center_frd` values.

The horizontal baseline/yaw read model is independent from the raw baseline and from physical
alignment acceptance. `CURRENT` requires a current selected P1/P2 GNSS baseline plus current P1
and P2 ATTITUDE. `PARTIAL` requires a current baseline and exactly one current board attitude;
that board's Baseline−Pn value remains visible. `STALE` retains finite last values and actual ages
without presenting them as current. `WAITING` means no finite non-zero horizontal direction or no
comparable attitude. The formulas are `Baseline−Pn=wrap180(azimuth(P1→P2)-Pn yaw)` and
`P2−P1=wrap180(P2 yaw-P1 yaw)`. These states are source availability/freshness only; there is no
GOOD/WARN/BAD angle verdict.

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
`PX4 yaw°` and `Δyaw°` remain backend diagnostic fields and are not operator-table columns. `WAITING`,
`STALE`, `PARAM 대기`, `PARAM 불완전`, and `전류 보정 대기` remain visible as corrected-yaw
states. The Pixhawk MAG control adds side-specific `PARAM APPLYING`,
`PARAM VERIFIED`, `VERIFYING`, `ACTIVE VERIFIED`, `STALE`, `TIMEOUT`, and `ERROR` evidence; it
does not create a global status key. Live-Pixhawk Measurement admission consumes the public
side-specific verification snapshot and records its selection provenance; no stream request,
firmware change, or persistent parameter schema is added.

## 13. Detection

Separate configured/model-loaded/runtime-running/target-present/output-fresh states where applicable.

## 14. Measurement

Candidate states: UNAVAILABLE, WAITING, VALID, DEGRADED, REJECTED, STALE, ERROR. These are hypotheses until mapped to existing measurement logic.

Rejection reasons must come from existing logic, not the UI.

## 15. Recording

Manual recording: IDLE → STARTING → ACTIVE → STOPPING → IDLE, with DEGRADED/ERROR diagnostics.
Measurement enters RUNNING immediately after its in-memory generation/time boundary is frozen.
Source states (`PENDING/WAITING_RESOURCE/LIVE/UNAVAILABLE/LEFT`) are independent and never own run
admission.

The compact post-run recording state is presence-only: `SAVED`, `PARTIAL`, or `FAILED`. Its detail
explicitly says playback integrity is not verified. Recorder, artifact or storage failure never
changes Measurement state or prevents the next run.

The Recording workspace also exposes an advisory protocol projection (not a new
runtime status key). It evaluates camera LIVE, RTCP/NTP SYNCED, detection RUNNING,
active calibration, and the selected Pixhawk/GNSS or checkerboard geometry evidence.
It returns `READY` only when all required checks pass; it never treats a command
acceptance as proof that a source is live. Storage state is observable but never a Measurement gate.

The Data Status stream projection exposes the authoritative JSONL path/state and the independent
same-stem CSV path/state. CSV `FAILED`/`RETRYING`/`DROPPED` is diagnostic evidence only; JSONL and
Measurement remain available and `Retry Storage` is the recovery action.

### 15.1 Latest Measurement XLSX

The Data workspace owns a local derived-artifact read model, not a new global runtime status key:
`NOT_CREATED`, `WAITING`, `GENERATING`, `READY`, `STALE`, `FAILED`, or `CANCELLED`. It includes the
target and published generation, output path, generation time, per-sheet row counts, issue count,
checkpoint settled/pending evidence, completion basis, and error. Request acceptance is not
publication success; only `READY` with an existing app-owned workbook is success.

`STALE` means a newer target or late matching row exists while the prior workbook remains usable.
Missing device/source rows are shown inside the workbook as zero rows/`#N/A`, not as a false export
failure. A long path/error is compacted in the fixed-height label and retained in its tooltip, so
status changes never move buttons, focus, selection, scroll, docks, or the window.

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
