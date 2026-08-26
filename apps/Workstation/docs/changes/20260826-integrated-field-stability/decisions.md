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

## 2026-08-26 — First implementation slice selected from real MAVLink A/B

- P1/P2 firmware, saved parameters, interface/link and UDP instances are correct, but their
  Ethernet instances show tx=0 and rising txerr before a known GCS peer exists.
- A 1 Hz GCS heartbeat from the exact bound 14550/14551 socket immediately restores both links.
- Implement explicit per-side peer seeding and heartbeat before inbound identity. Only an
  expected-sysid HEARTBEAT may verify identity; arbitrary telemetry cannot unlock commands.
- Retain the 10-minute qualification and do not start automatic bulk parameter refresh before LIVE.

## 2026-08-26 — Remote PX4 boot rearms only the ephemeral stream profile

- Treat a confirmed per-side `ATTITUDE.time_boot_ms` epoch regression as remote reboot evidence even
  when the Workstation PID, UDP socket and host-listener generation are unchanged.
- Require two increasing low-epoch samples after a regression beyond the 2-second reorder tolerance,
  including a rapid reboot before 10 seconds of uptime. Ignore regressions up to and including 2
  seconds and natural uint32 wrap; exclude delayed prior-epoch ATTITUDE from current telemetry.
- Reset that side's identity/10-minute qualification and Stock-profile request marker only. The next
  expected-sysid HEARTBEAT sends one serialized profile burst through the existing subscriber owner.
- Do not emit the persistent setup signal or send reboot, shell, parameter or `MAV_2_*` configuration
  commands. Preserve the healthy peer and all persistent configuration markers.

## 2026-08-27 — Pixhawk progress cannot block receive-timer completion

- Queue subscriber status/shell and connection completed/failed/cleanup callbacks explicitly to
  the Qt GUI owner. Start the subscriber receive timer only on its QObject owner thread.
- Generic progress status may log and coalesce side-action/calibration refresh, but it does not own
  topology-button state. Completion, failure and disconnect owners update Connect/Disconnect text.
- Preserve button text and the semantic `role` property without synchronous QStyle
  `unpolish()`/`polish()`. Accessibility remains enabled; external accessibility saturation is not
  permission to sacrifice GUI liveness.
- Bound successful RTCM progress text to one message per side every five seconds while preserving
  every error and the exact worker-owned frame/packet/byte/failure counters.

## 2026-08-27 — Command reservation owns heartbeat; historical replay owns no commands

- Match the current PX4 Stock firmware parameter surface: automatic calibration refresh reads
  `CAL_MAG0_*` through `CAL_MAG3_*` only (124 total calibration parameters with the attitude set).
  Keep arbitrary names available to an explicit generic parameter caller.
- Keep PARAM/shell receive ownership and the complete per-side send reservation. The command owner
  services GCS heartbeat from its bounded receive poll by re-entering that same lock; the GUI timer
  never waits for a busy command lock. RTCM remains excluded until the reservation ends.
- Record unmatched telemetry liveness at ingress and replay it FIFO with its original timestamp.
  Suppress setup/profile/ATTITUDE-recovery actions only for a historical HEARTBEAT; a new live
  HEARTBEAT retains the existing one-shot behavior.
- Do not change the deliberate post-command qualification reset in this slice.

## 2026-08-26 — Camera and video start success split by external postcondition

- Camera QThread/process creation does not set `camera_connected` or Measurement RUNNING.
- Each accepted side must emit a current-generation decoded frame. After the bounded timeout, one
  verified side may continue as partial; no verified side retires the start to ERROR and stops its
  automatic recording owner.
- MP4 is a separate best-effort state. Positive file growth is required for video RUNNING, but video
  PARTIAL/FAILED does not invalidate a frame-ready Measurement or disable the next Start.

## 2026-08-26 — STUS is an ordered command and VERIFIED is continuously revocable

- For a dual target, send Left STUS first and require its exact `OK` response before sending any
  Right STUS request. A timeout, exception, empty response or non-OK response leaves Right untouched.
- Keep the recording interlock in both the visible command and backend controller; no automatic
  STUS is introduced.
- After reconnect/fresh-SR warmup, keep a 1 Hz quality evaluation instead of stopping at the first
  VERIFIED result. Select the freshest receive-close frame pair rather than the healthiest old pair.
- Advance stored per-frame SR age with current monotonic and wall observations. Missing/regressed
  SR, stale current frames/SRs, pair delta over 50 ms, SR-PC over 100 ms, or clock-rate error over
  0.5% revokes VERIFIED immediately. Keep the SR-age limit configurable; do not silently clamp or
  migrate an operator value. The approved field run must visibly Apply 7.0 seconds and record it.

## 2026-08-27 — One manual STUS action may make one evidence-gated correction

- Supersede “no automatic STUS” only inside the already explicit Set camera clock action: no
  startup, periodic or independent automatic STUS exists, but that one action may perform one
  bounded feedback correction before becoming terminal.
- Before either round, require at least two of three exact-OK HIWS samples from both sides. Treat
  250 ms as a high-latency warning, not a rejection; accept only RTT at or below 2.0 seconds.
- Use only signed `camera SR NTP - PC SR receive UTC` feedback. Deduplicate repeated frames by raw
  SR identity and require three stable, post-ACK, replacement-generation `rtcp-pad-probe` SRs per
  side. Never actuate from frame receive delay, absolute max offset, pair delta alone, poll fallback
  or STUS ACK duration.
- Compute `next lead = previous lead - median signed SR-PC offset`. Permit signed lead, but fail
  rather than clamp when feedback or target exceeds ±2.0 seconds. Allow two total rounds and 90
  seconds for the complete action; second-round nonconvergence authorizes no third mutation.
- Preserve exact Left `OK` before Right in every round, recording and camera-resource interlocks,
  and fail closed on either-side preflight/ACK failure, stale or unstable evidence, timeout,
  cancellation or late completion. Reject raw SDK `STUS` so it cannot bypass these owners.
- ACKs alone record `pending-verification`. Three stable SRs plus all hard limits record `success`;
  a later continuous-monitor or stream-generation reset records `revoked`.
- A real exact STUS `OK` arrived 8.876 seconds after command, so the generic 5-second SDK timeout is
  not the STUS apply contract. Wait up to 15 seconds only for a STUS mutation response, capped by
  the remaining 90-second action deadline; retain configured timeout for HIWS and all generic SDK
  work. Timeout remains terminal and never authorizes the next side or another round.
- On 2026-08-27 the user explicitly classified sustained STUS failure as `Critical`. A failed or
  unverified sync state prevents final Measurement-result acceptance even though lifecycle Start
  remains available for diagnostic capture; the packet and final soak cannot close on degraded
  timestamp evidence.

## 2026-08-26 — Decoder truth and fallback ownership are per side

- Treat GStreamer `PLAYING` as candidate launch evidence, never as active GPU evidence. Only a
  decoded current-generation frame may set the side's effective decoder.
- Classify RTSP/source/parser, mux/filesink, decoder-health and generic-pipeline failures separately.
  Only a direct NVDEC health failure, or missing decoded output while current RTP input proves the
  source boundary remains active, may authorize Auto software fallback.
- Preview-only fallback stays inside the failed side worker. Recording fallback finalizes/stops the
  failed side and records a software override for its next start; it does not reopen the same MP4.
- Never restart the healthy side for a peer decoder failure. Mark an intentional failed-side exit as
  terminal so the process supervisor cannot misclassify it as a restartable crash.

## 2026-08-26 — Shutdown completion requires verified cleanup and durable video outcome

- Preserve each camera's full 12-second cooperative stop window, including the nine-second MP4 EOS
  wait, measured from that camera's actual stop request when earlier lifecycle steps delayed it.
- Treat session-save and both HR/OZ capture workers as close gates. A force request is escalation,
  not completion; a live child, QThread, socket, or native shared-memory ring keeps close pending.
- Keep the camera QThread alive while terminal/finalization IPC can still drain. Do not forget a
  child process handle that remains alive after kill/join.
- Enqueue per-side EOS success or synthesized failure evidence before settings flush. Keep a failed
  enqueue retryable, sweep retired rings after their leases drain, verify resource postconditions,
  then close DataStorage as the final durability barrier.

## 2026-08-27 — Corrupt H.264 never crosses an unproven recovery boundary

- Require a parsed Annex-B NAL type 5 IDR at startup and after RTP/SSRC discontinuity. The encoded
  recording tee opens only at that IDR; Preview/YOLO/native evidence open only when the decoder
  returns that exact PTS. Parser `non-delta` and NAL1 intra pictures are insufficient.
- A zero-IDR side becomes explicitly unavailable after 30 seconds without automatic reconnect or
  peer-side disruption. Same-generation queued frames cannot clear that terminal latch.
- The user approved SDK setting changes and one sequential reboot per FW 2.18.72 camera. Reboots
  proved IDR is non-persistent; restoring `GETH=TRUE`, `0/1/30/1` and reapplying IDR while RTSP was
  active still produced zero NAL5 on both devices. Both finish restored and healthy; no firmware
  install or alternate re-encode path is approved.

## 2026-08-27 — Finalized H.264-copy MP4 is usable only after asynchronous byte validation

- Positive growth and EOS/container finalization are provisional. Full validation runs outside the
  camera child/12-second stop budget in the package worker against immutable backend and live
  terminal evidence.
- Require first-VCL IDR, H.264 AU/MP4 sample/software-decoded equality, supported reference
  continuity, positive contiguous timing including final `stts`, and sane frame-rate/time-base.
  Unsupported syntax or incomplete evidence fails closed; no missing frame is reconstructed.
- Live RTP/IDR/suppression failure and recording-start backend provenance cannot later be overwritten
  by a clean byte-only probe or a changed UI setting.

## 2026-08-27 — Undefined MAVLink numerics are null evidence, not callback exceptions

- Normalize NaN/±Inf optional, scaled and custom telemetry to JSON null while persisting exact
  `MESSAGE.field` paths and non-finite classes. A bad sample timestamp falls back to host receive
  time with explicit provenance; strict storage rejection remains defense in depth.
- Full Pixhawk disconnect stops the GUI-owned receive timer on its owner before worker-thread
  transport close. Worker QThreads never stop that QTimer directly.

## 2026-08-27 — Camera and Pixhawk traffic remain on separate NICs

- Treat the former shared broadcast domain as the strongest reproduced trigger discriminator: dual
  RTSP load caused persistent dual PX4 service silence there, while both isolated adapters passed
  direct load and application ten-minute gates.
- Current routing owns cameras on `enx588694fce07e`/`10.0.0.11` host routes and P1/P2 on RTL8156B
  `enx0023a40a2e39`/`10.0.0.10/24`. Neither supplies a default route. Isolation does not by itself
  prove the PX4 #25956 internal IOB mechanism.

## 2026-08-27 — NTRIP transport/injection PASS is distinct from RTK Fixed

- Accept NTRIP transport only from raw HTTP 200, checksum-valid GGA, CRC-valid station/MSM RTCM,
  continuous wire duration and per-side injection failure counters. A TCP connection alone is not
  success.
- The isolated replacement topology passed normal Stop/reconnect and 636.5 seconds with dual
  failure-zero injection while P1/P2 remained LIVE. Indoor rover `fix_type=0` is not RTK convergence;
  open-sky Fixed/accuracy remains separate.
- Keep the operator password process-local unless secure-store opt-in is explicit. Current keyring
  remains `NOT_SAVED`; quitting the app cleared the session credential.

## 2026-08-27 — Abnormal exit cleanup has tracker and startup recovery layers

- Name every native camera ring `sasl_ws_native_<creator-pid>_<16-hex-token>` and keep spawned
  camera attachments registered with the shared multiprocessing tracker. Cooperative ownership
  still closes/unlinks explicitly; tracker EOF owns the normal `SIGKILL` fallback.
- Treat tracker reclamation as insufficient for the restart contract. Before Qt construction, scan
  only the strict Workstation prefix and unlink only a regular entry owned by the current UID whose
  embedded creator PID is dead. Never sweep arbitrary `psm_*`, malformed names, or live/current PID
  entries; PID reuse therefore fails conservatively by retaining rather than deleting.
- Historical/incomplete run data remains inert. A restart does not adopt it or gate `측정 시작`; a
  new operator action creates a distinct runtime generation, while the previous incomplete rows stay
  append-only evidence.

## 2026-08-27 — Native descriptor bytes must be independently reconstructable

- Persist every field included in `frame_descriptor_sha256`, including app-receive `timestamp_s`, in
  the frozen detection metadata. Equality between two copied hashes is not independent evidence.
- Audit one exact runtime without importing its producer: check run-scoped origin linkage, FrameKey
  redundancy, descriptor canonical bytes, PNG structure/decompression, file bytes/hash/path and pair
  ACK. A historical missing timestamp may come only from that pair's ACK-bound artifact row.

## 2026-08-27 — BLOCKED methods do not own Measurement admission

- Missing/mismatched K,D, R,t, correspondence, live Pixhawk/GNSS or verified camera timing remains a
  per-method/result blocker. It does not disable the lifecycle Start used to collect diagnostic
  evidence. Every attempted method still records a terminal BLOCKED outcome; no result is promoted.

## 2026-08-27 — Legacy Record starts real camera recording or fails visibly

- Reserve `10.0.0.11/32` for the camera NIC in both application defaults; `.10/24` belongs to the
  isolated Pixhawk NIC and must not be recreated on the camera interface.
- Treat Legacy `세션 기록 시작` as a two-camera MP4 request even when Preview is disconnected.
  Start both record-capable streams immediately and mark both expected before first frame. If the
  request is synchronously rejected, restore the Start state and retain a retryable failure.
- MP4 existence/EOS remains separate from usability. Zero-IDR bytes may finalize but must stay R7
  FAIL under the strict validator.

## 2026-08-27 — Canonical venv owns the Torch/colcon setuptools intersection

- Keep the repository-root Python 3.10 venv with system site packages, but explicitly require
  `setuptools>=77,<80`: Torch 2.13 requires the lower bound and Ubuntu/ROS colcon requires the upper.
- A fresh same-host checkout/venv and isolated HOME are valid manifest-recreation evidence, but do
  not replace a different clean Ubuntu OS/VM. Preserve that distinction in R10.
- Historical calibration/change evidence may retain captured absolute paths. Active configuration,
  scripts and operator examples may not; the repository portability gate must report zero.

## 2026-08-27 — External operator models remain untracked and manifest-verified

- Decision: Keep both approved YOLO model binaries outside Git. Track their filename, byte count,
  SHA-256 and provisioning contract, and make each application runtime check fail closed when its
  default model is missing or does not match.
- Reason: Repository policy explicitly excludes large downloaded model files, but a relative path
  alone falsely implied that Legacy's ignored `yolov8n_uav.pt` was bundled. A clean commit archive
  proved the bytes were absent.
- Consequence: A clean deployment must explicitly provision Workstation and Legacy model artifacts.
  Custom Legacy model selections preserve their existing presence-only contract; the approved
  default is the hash-locked reproducible path.
