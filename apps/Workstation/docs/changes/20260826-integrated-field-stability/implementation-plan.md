# Implementation Plan

## Approval status

Approved on 2026-08-26. Implementation may change only the integrated scope described here; every
new device setting, firmware change, destructive recovery or unrelated feature needs new approval.

## Slices and dependencies

1. Freeze repo/runtime/device/settings baseline and consolidate prior packet evidence.
   Before any StereoWIRIS Legacy hardware run, prove launch/connect/Preview/record cannot auto-apply
   the conflicting repo encoder profile.
2. Run independent short probes: RTSP/NVDEC, MAVLink, NTRIP/RTCM, STUS, admission/restart,
   MP4/evidence, GUI/resource/shutdown.
3. For each failure, prove the smallest layer, add a regression, implement the minimal slice, and
   rerun its focused tests before any dependent path.
4. Run 10-15 minute problem qualifications plus three repeated record/restart workflows.
5. Run three independent 10-minute cold-start cycles.
6. Run one 60-minute full-load soak and audit all durable evidence.

## File and component map

- Camera/media/time: `camera_stream_controller.py`, `workstation_core/yolo_stream.py`,
  `workstation_core/native_frame_ring.py`, `workstation_core/detection_pair_coordinator.py`,
  WIRIS control/time-sync owners and camera tests.
- MAVLink/NTRIP/RTCM: `device_runtime_workers.py`, `device_runtime_controller.py`,
  `gnss_runtime_controller.py`, `workstation_core/mavlink_liveness.py`,
  `workstation_core/rtcm_pipeline.py` and focused tests.
- Measurement/storage/recording: `measurement_run_controller.py`, `recording_controller.py`,
  `data_storage_controller.py`, native evidence/store modules and lifecycle tests.
- UI/status: existing Camera/GNSS/Data/Diagnostics workspaces and status reducers only when an
  observed contract defect requires them.
- Durable evidence: this packet plus affected current product documents after a contract changes.

## Migration and rollback steps

No data migration. Back up/recreate the canonical venv only if its doctor requires it. Code changes
remain independently revertible; runtime/device settings are snapshotted before any separately
approved experiment and restored/read back afterward.

## Stopping conditions

- Stop a candidate immediately on a failed short or qualification gate.
- Stop source edits on a new scope/ownership/device-setting requirement and request approval.
- Ask the user for the named physical action when USB, power-cycle, motion target or antenna change
  is required.
- Stop before a Legacy hardware run if any path can apply the stale `0/1/90/1` profile; do not use
  runtime experimentation to discover this mutation.
- Do not begin the final soak with any required `FAIL`, `NOT_RUN` or unresolved `BLOCKED` slice.

## Approved STUS continuity implementation order — 2026-08-27

1. Add backward-compatible Manual/Auto interval settings and pure scheduler/epoch contracts.
2. Add a bounded decoder-free RTCP observer whose own session may restart after STUS without
   restarting CameraPane, YOLO or MP4 ownership.
3. Move closed-loop feedback and continuous clock health to observer SR evidence; keep Frame timing
   and Media truth separate and fail closed when observer evidence is unavailable.
4. Allow manual/scheduled STUS during Measurement/YOLO/recording, create append-only operation and
   clock-epoch evidence, and block only timing-dependent results across an epoch transition.
5. Project mode, seed requirement, next due, command phase, clock state and frame/media state into
   the existing Time / Sync workspace with stable geometry.
6. Verify focused contracts, worker failure isolation, persistence/migration, active-work continuity,
   complete Workstation regression, then real 25-minute Manual and 70-minute Auto field gates.
7. Treat signed per-camera SR-PC as diagnostic evidence only. Drive convergence and continuous Clock
   quality from fresh/stable Left/Right relative RTCP mapping at a 100 ms bound; independently audit
   actual matched-frame deltas over the field window while allowing occasional unmatched/outlier
   frames.

## Proposed app-receive distance fallback slices — implementation not authorized

1. Add a pure timing-basis decision/state transition with manual-seed, Clock status, logical epoch,
   recovery and basis-generation contracts.
2. Capture exact integer app-receive monotonic ns at the decoded appsink boundary; preserve current
   float compatibility and all NTP/RTP/SR metadata.
3. Make the existing pairing coordinator use one explicit basis, flush on basis/epoch changes,
   hard-bound fallback pairs to 100 ms and remove hidden NTP-present drift behavior.
4. Freeze basis/reason/generation into `DetectionSnapshot`, `FrozenMeasurementPairSnapshot` and
   every method evaluation. Leave geometry, attitude/GNSS association and coordinate math intact.
5. Add authoritative storage schema v4 fields; make compact `sync_delta_ns` selected-basis truth and
   retain separately named app/NTP values with backward-compatible reads.
6. Project NTP VERIFIED, APP RECEIVE DEGRADED, transition wait and recovery through the existing
   Measurement/Time surfaces with stable geometry.
7. Verify pure logic, coordinator concurrency, cross-epoch/basis rejection, exact sentinel
   persistence, legacy read compatibility, UI state sequence, restart/shutdown, full regression and
   a real long-run drift→fallback→recovery workflow.

Stopping conditions: do not implement before the fallback result-validity decision and explicit
approval. Do not broaden this slice to calibration, Legacy, clock correction, exposure-time claims
or geometry algorithms.

## STUS cadence reliability evaluation — current priority

1. Keep Auto mutation disabled; use the 30-minute holdover artifact to compute cadence-only drift.
2. Repair/verify one bounded manual cycle first: collect three post-ACK SRs over the operation while
   applying the 7-second freshness limit only to the latest SR, restart only a stale observer side,
   and recenter relative correction targets within ±2 seconds.
3. Require five consecutive manual cycles to finish with absolute Left/Right delta ≤100 ms, target
   ≤50 ms, no cycle worsening above the starting delta, no stream/YOLO/MP4 restart and complete
   cleanup/evidence.
4. Only after step 3, run a 30-minute stress qualification at 5-minute Auto cadence (six due
   cycles). Stop on the first preflight/ACK/observer/phase failure or >100 ms final delta.
5. Derive the field interval rather than assuming the stress interval:
   `T_max = (100 ms - post-STUS residual - safety margin) / relative_rate`.
   Compare 5/10/15/30-minute candidates and then run the selected interval through a longer soak.

The fallback amendment above remains deferred and is not part of these source slices.

## Approved app-receive-authoritative implementation slices — software complete

1. Add exact integer app-receive monotonic capture at every supported decoded-frame backend and
   preserve float compatibility from that source.
2. Replace live timing-basis selection with a fixed pure `app_receive_monotonic` policy; remove
   NTP/Clock/epoch inputs from distance-pair decisions without changing other gates.
3. Hard-bound selected app pairs to 100 ms, preserve bounded latest buffers/reservations and record
   explicit drop reasons. Verify STUS transition cannot interrupt or relabel app pairing.
4. Freeze authoritative basis and exact app timestamps through DetectionSnapshot, pair identity,
   MeasurementAttempt and every method worker.
5. Add schema-v4 selected/app/NTP fields; project compact `sync_delta_ns` from selected app delta and
   keep NTP record-only with schema-aware v3 compatibility.
6. Replace the live basis selector with stable read-only app-authoritative status and expose app/NTP
   deltas without degraded classification.
7. Update current product docs, generated delegates if affected, and README only after approval.
8. Verify pure sentinel logic, backend parity, concurrency/backpressure, persistence readback,
   schema compatibility, UI states, STUS continuity, full regression and target-hardware app-pair
   timing under sustained load.

The user approved this exact amendment at `2026-08-27T14:31:06+09:00`. Slices 1–7 and the software
portion of slice 8 are complete: focused tests pass 244 cases and the full Workstation suite passes
1515 cases with 1 skip. The target-hardware portion of slice 8 remains NOT_RUN because both current
camera streams emit zero NAL5 and therefore cannot supply decoded pairs. The independent STUS
cadence evaluation may continue later but no longer owns live distance validity.

## Proposed indoor 60-minute experimental slices — awaiting approval

1. Add a default-off process-local recovery policy and pure AU classification. Strict NAL5 remains
   the default and all non-GStreamer backends remain outside this experiment.
2. Admit only parser `non-delta` NAL1 AUs containing current SPS/PPS with valid PTS and clean flags;
   persist/emit the exact recovery basis and relatch on every later discontinuity.
3. Keep direct-copy MP4 integrity strict and visibly unverified/failed under experimental recovery;
   add no re-encode, worker, queue or device mutation.
4. Add fixed-width existing-surface status text, focused negative tests, dependent camera/
   Measurement/recording tests, complete Workstation regression, launcher and shutdown checks.
5. Stop all current owners, launch once with the experiment flag and process-local NTRIP credential,
   then run the five-minute camera/MAVLink/NTRIP/recording/UI/resource gate.
6. On gate PASS, start one 3,600-second monitor with checkpoints every 300 seconds. Keep RTCP Clock
   observation active but send no STUS mutation. Stop early on a required failure.
7. Stop/drain/finalize, independently inspect JSONL/Parquet/native evidence and both MP4s, verify no
   child/socket/ring residue, restore a normal strict-mode launch and record final status.

No implementation or hardware run is authorized until the approval summary above is explicitly
approved. A camera motion target and the user-supplied session credential are preflight inputs, not
persisted configuration.

### Prepared automation/control map

- AT-SPI application/frame identity: `__main__.py` / `StereoWIRIS Workstation`.
- Intent actions use enabled/showing instances of `Connect cameras`, `Connect Pixhawk`,
  `Start NTRIP` and `측정 시작`; Stop/Disconnect/Quit are resolved from the same owner after state
  transition rather than by screen coordinates.
- The external oracle reads the existing Diagnostics model rows `camera.left`, `camera.right`,
  `pixhawk.left`, `pixhawk.right`, `gnss.ntrip`, `gnss`, `detection`, `synchronization`,
  `measurement`, `recording` and `fps`, plus status-bar text. It never polls or removes product
  queue items.
- Process/resource evidence uses exact Workstation/child PIDs, `/proc`, `ss`, NIC counters,
  `nvidia-smi`, filesystem free space and run/artifact file sizes. Secret-bearing widgets,
  environment values and process environments are never serialized.
- Proposed implementation path for the low-rate owner is `tools/qualify_indoor_60m.py`; each row is
  append-only JSONL under this packet's `evidence/`, with a separate terminal summary. The tool
  exits nonzero on an early required failure and always invokes the bounded stop/cleanup audit.

### Outdoor approval update

The user approved this slice and changed the execution location to outdoors at
`2026-08-27T19:40:48+09:00`. Rename the monitor to the location-neutral
`tools/qualify_integrated_60m.py`. Add P1/P2 fix type, satellites/accuracy and sustained RTK Fixed
to preflight/checkpoint/final evidence. Implementation and the real run are now authorized.

## Approved manual-only PX4 PARAM slice — 2026-08-27

1. Remove the LIVE-status automatic calibration-refresh hook and its per-session marker state.
2. Preserve explicit full/MAG/attitude refresh and post-reset/apply/save readback routes unchanged.
3. Add a regression proving repeated `LIVE` status refresh sends no PARAM work, plus preserved
   command-routing tests for P1/P2 manual refresh.
4. Update current product/operation documents so `PARAM SYNC` is explicit-command-only and old
   automatic field artifacts remain labelled historical.
5. Run focused MAVLink/Pixhawk/UI tests, then the complete Workstation suite and launcher check.

No settings migration, device operation, UI geometry change, or hardware PARAM request is needed.
Rollback is the isolated automatic hook and marker state.

## Approved 60-second operational MAVLink qualification slice — 2026-08-27

1. Change the single liveness-owner default qualification threshold from 600 to 60 seconds.
2. Remove hard-coded `/10:00` and 600-second UI fallbacks; render the snapshot-owned required
   duration dynamically without changing geometry.
3. Update USB recovery/operator copy and evidence contracts from ten-minute to one-minute
   requalification.
4. Make the integrated monitor require `LIVE` after 60 seconds while keeping 10/60-minute elapsed
   duration as separate stability evidence.
5. Add exact 59.999/60.000-second, stall recovery, reboot, explicit command, stale/stall and
   no-automatic-PARAM regressions. Run focused UI/RTCM/device tests, then the full suite and launcher.

No setting, migration, protocol packet, device mutation, worker, queue, coordinate transform or
measurement formula changes. Rollback is the threshold plus factual presentation/documentation.

## Approved Mountain GNSS 3D-fix slice — 2026-08-28

1. Replace only Mountain translation observability's both-`RTK_FIXED` check with numeric
   `fix_type >= 3` on both sides.
2. Retain the same-burst finite-position and finite non-zero P1-to-P2 NED baseline requirements;
   preserve all image-correspondence rotation rank/inlier/condition gates.
3. Persist the numeric fix types in candidate quality evidence and add an explicit non-fixed
   warning so a usable 3D/RTK-Float Mountain outcome is `ESTIMATED`.
4. Prove sub-3D rejection, 3D/RTK-Float translation observability, next-Measurement Draft freeze,
   method readiness, representative-result stability, and no settings/schema/UI-layout change.
5. Update Workstation usage, workflow, recording-schema and change-packet documentation; then run
   focused and complete regression checks.

Rollback is isolated to the Mountain fix gate and its evidence/warning fields. Existing rows,
global GNSS settings, `pixhawk_rtk_live`, coordinate transforms and device behavior are unchanged.

## Proposed Mountain direct-suggestion slices — implementation not authorized

1. Add a pure owner-side Mountain proposal filter that validates current frame identity, shared
   exclusions, paired point count, bounds, uniqueness and distribution independently of confidence.
2. Extend optional acceptance to structurally usable LOW_CONFIDENCE Mountain proposals while
   keeping Horizon's GOOD-only path unchanged; persist filtered provenance.
3. Extend Mountain Solve payload/owner resolution so empty manual input can use the declared latest
   usable suggestion directly. Preserve complete manual precedence and reject partial manual input.
4. Add explicit quality/usability status and tooltip text in the existing group without layout or
   control additions. Verify exclusions, focus, selection, scroll and frozen-view state.
5. Cover stale/new Capture, malformed/excess exclusions, fewer than four, duplicate/collinear,
   LOW_CONFIDENCE success/final-solver block, busy/duplicate commands, manual precedence,
   persistence, next-Measurement freeze and unchanged Horizon/Apply/primary behavior.
6. Update the user-approved Workstation README and current UI workflow/command/status documents;
   run focused, expanded and complete Workstation regression plus `./run.sh --check`.

Stop before source edits until the recommended visible workflow and README target are explicitly
approved. No new worker/resource or hardware gate is required.

## Approved Mountain Use suggestion quality-separation slices — 2026-08-29

1. Represent Mountain structural usability independently from detector quality while preserving the
   existing `GOOD`/`LOW_CONFIDENCE` evidence.
2. Revalidate current frozen identity, paired exclusions, bounds, pair count, uniqueness and
   distribution in the controller; copy only the filtered validated points.
3. Enable the existing `Use suggestion` control for structurally usable Mountain proposals
   regardless of `GOOD` versus `LOW_CONFIDENCE`; keep Horizon `GOOD`-only.
4. Preserve explicit status/provenance and update Workstation README plus current UI contract docs.
5. Run focused backend/UI/persistence tests, dependent optical-pose regressions, the complete
   Workstation suite and `./run.sh --check`.

Direct Solve fallback and all automatic copy/Solve/Apply behavior remain outside this approval.

## Approved Horizon Use suggestion quality-separation slices — 2026-08-29

1. Add a pure Horizon copy validator for current-frame native endpoints, bounds, exact two-per-side
   count and the existing 25%-width separation contract, with no detector-quality input.
2. Project Horizon `copy_eligible/copy_block_reason` independently of `state/eligible`; revalidate
   and normalize the same endpoints in the controller before acceptance.
3. Enable the existing `Use suggestion` control from structural readiness, keep detector quality
   text and accepted warning/provenance, and add no control or geometry.
4. Preserve Mountain behavior, manual Horizon, explicit Solve/Apply, stale/busy handling and
   append-only evidence. Update the confirmed Workstation README and current UI contracts.
5. Run focused backend/UI/persistence tests, dependent Alignment/command regressions, the complete
   Workstation suite, visual state comparison and `./run.sh --check`.

## Approved Settings-preferred live-distance timing slices — implementing 2026-08-31

1. Normalize missing/invalid settings to app receive and legacy strict-required to RTCP/NTP
   preferred-with-fallback; freeze the request at each Measurement boundary.
2. Add one pure normalized requested/effective/fallback policy with generation transitions.
3. Route Settings Apply/Save through that owner; use app receive as the default and mandatory error
   fallback, applying active-run changes at the next Measurement and flushing candidates on idle
   policy or runtime effective-eligibility changes.
4. Make coordinator, frozen snapshot, method fan-out and UI consume the same policy snapshot; remove
   all connection-state basis overrides and duplicate widget reads.
5. Persist requested/effective basis, fallback reason and distinct app/NTP deltas additively while
   preserving schema-aware historical reads.
6. Fix the independent successful-preview `input_integrity_text` scope defect at the frozen-attempt
   boundary and add its regression without changing timing policy.
7. Verify app default, healthy preferred source, every fallback reason, recovery, 100 ms edges,
   buffer-generation isolation, worker validity, persistence, restart, UI stability, full suite and
   the documented launcher.

Healthy RTCP/NTP owns pair selection only with VERIFIED/current/same-epoch finite evidence inside
the configured bound. All RTCP timing faults fall back to exact app receive without a timing-only
validity downgrade. The user approved this scope and source work on 2026-08-31.

Implementation status: slices 1–6 are complete. Slice 7 has 299 focused software/UI/persistence
tests and the visual state check passing; complete regression and launcher verification remain.
