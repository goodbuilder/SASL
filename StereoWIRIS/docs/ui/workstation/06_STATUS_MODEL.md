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

Potential dimensions: connection, RTSP availability, decoder, frame reception, frame age, FPS, preview. Aggregate LIVE only if actual frames are confirmed.

## 11. Pixhawk Status

Potential dimensions: transport/connection, heartbeat/telemetry, attitude availability, attitude age, source. ROS2/MAVLink semantics must not be collapsed without evidence.

## 12. GNSS / RTK

Preserve source semantics: NO DATA, NO FIX, FLOAT, FIX, STALE, etc. Do not substitute generic OK.

## 13. Detection

Separate configured/model-loaded/runtime-running/target-present/output-fresh states where applicable.

## 14. Measurement

Candidate states: UNAVAILABLE, WAITING, VALID, DEGRADED, REJECTED, STALE, ERROR. These are hypotheses until mapped to existing measurement logic.

Rejection reasons must come from existing logic, not the UI.

## 15. Recording

Candidate lifecycle: IDLE → STARTING → ACTIVE → STOPPING → SAVED/IDLE, with ERROR where applicable. Exact mapping must be validated against current recording implementation.

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

## 21. Startup / Shutdown

Use UNKNOWN vs DISCONNECTED according to actual knowledge. Expected STOPPING/DISCONNECTING/SAVING transitions are not errors.

## 22. Summary

> Display evidence, not optimism.

Implementation note: `BackendStatusAdapter` is the aggregate status projection and
`EvidenceAdapter` supplies detailed read-only values. A command being accepted does not
make a device live, a recording saved, or a measurement valid; the corresponding source
status and age/clock evidence must confirm it.
