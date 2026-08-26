# Decisions

Append accepted decisions chronologically. Supersede entries rather than deleting history.

## 2026-08-26 — Integrated goal approved

- Combine the already approved camera, runtime, MAVLink, NTRIP, sessionless Measurement, recording
  and evidence work into one field qualification and defect-repair goal.
- Use short independent gates first. Only a candidate that passes all gates receives the final
  60-minute full-load soak.
- Exclude cross-app rig ownership and Diagnostics log-filter implementation.
- Keep device/firmware/configuration mutation outside scope unless separately approved.
- Treat STUS pair delta 50 ms, SR-PC offset 100 ms and clock error 0.5% as hard limits. Target SR
  age is 7 seconds; the current 10-second setting is a baseline mismatch to resolve with evidence.

## 2026-08-26 — Lifecycle admission and method readiness remain separate

- Old Session/spool, storage failure, MAVLink, GNSS/NTRIP and STUS do not disable Measurement Start.
- Only camera/resource ownership may queue the lifecycle transition.
- Current-zoom intrinsics, stereo R,t, correspondence, attitude, GNSS/RTK and synchronization gate
  only methods that require them; missing inputs produce explicit invalid/rejection reasons.
