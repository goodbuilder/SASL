# Impact Analysis

## Preserved invariants

- Camera encoder/transport and PX4 firmware/parameters are unchanged without separate approval.
- One resource owner, bounded work, exact identities, append-only evidence and truthful status.
- Spatial coordinate contracts and calibration/triangulation math are unchanged.
- Existing data and manual recovery paths remain available.

## Affected layers and components

Potentially affected: camera/media/time workers, MAVLink/NTRIP/RTCM workers and reducers,
Measurement/recording/storage coordinators, existing status projections, launch/runtime checks,
tests and product documentation. Actual source edits are selected only after reproduction.

## Feature dependency, ownership, and coupling changes

No new ownership transfer is approved. The goal verifies existing explicit coordinators and removes
only observed hidden coupling: GUI image delivery to control/evidence, stale side to RTCM routing,
old persistence state to new Measurement admission, and requested decoder to assumed actual state.

## Worker CPU/GPU resource impact

Baseline and candidate use the same named workload. Track process/thread topology, hidden decoder/
inference threads, CPU/GPU/memory, queue depth/age/drop, throughput, UI latency and thermal state.
No affinity, priority, quality/rate or worker-count change is preapproved.

## Compatibility and migration

No schema/data migration. Current settings are read in place. A canonical venv rebuild may be an
operator deployment step but may not silently consume an app/System/Simulation fallback.

## Rollback and recovery

Each defect fix has a narrow regression and reversible patch. Device experiments require pre/post
readback and restoration. If rollback restores false success or unbounded behavior, the feature is
left visibly unavailable rather than silently degraded.

## Risks and assumptions

The camera sender may not supply missing reference payload/real IDR, and PX4/NuttX may deadlock its
Ethernet MAVLink task. Real field evidence can therefore prove an external blocker rather than yield
an app-only solution. Simultaneous high-bitrate recording may expose thermal/storage/resource faults
not seen by software tests.

## Proposed app-receive fallback impact — not yet approved

- Pairing owner: extend the existing detection-pair policy; create no second matcher or worker.
- Clock dependency: consume only the public Clock state/seed/epoch snapshot. Clock/STUS remains its
  own owner and cannot be relabelled by Measurement fallback.
- Frame boundary: use host `CLOCK_MONOTONIC` captured immediately after decoded appsink pull and
  before frame copy/YOLO. It is comparable across the two same-host camera processes but includes
  per-side RTSP/network/decode delay and is not exposure time.
- Measurement: pair selection and timing validity/provenance change; disparity, triangulation,
  calibration, attitude/GNSS association and NED/FRD/RDF contracts do not.
- Persistence: additive new-row schema fields and a new schema version; no old row/file rewrite.
  Compact `sync_delta_ns` must mean the selected basis, while app and NTP deltas remain separately
  named. Existing readers retain legacy fallback when the selected field is absent.
- UI: existing Measurement/Time status text only; no geometry or navigation change.
- Resources: no new camera, RTSP, decoder, YOLO, queue, process, thread, CPU or GPU owner. A pure
  policy decision and additive serialization have negligible load.
- Failure isolation: bad Clock evidence may degrade pair timing but cannot stop imagery/recording;
  app delta over 100 ms, mixed basis generations or clock-epoch transition still produces no pair.
- Rollback: disable the proposed health-aware fallback and keep additive fields readable; never
reinterpret or delete rows already written with `app_receive_monotonic` provenance.

## Superseding fixed app-receive impact — awaiting approval

- Live Measurement no longer depends on Clock/STUS for pair timing or result validity. This removes
  a dependency rather than adding an owner.
- Existing camera process timestamps become exact integer boundary evidence. No new worker, queue,
  RTSP session, CPU/GPU workload or device mutation is introduced.
- NTP/RTCP observation, scheduler and logical epochs continue unchanged for Time / Sync and storage;
  they become record-only inputs to live distance rows.
- Pair identity, snapshots, canonical hashes, authoritative JSONL, Parquet/review projections and
  readers change additively. Schema-aware readers prevent historical RTCP `sync_delta_ns` from being
  reinterpreted as app time.
- Distance geometry, calibration algorithms, attitude/GNSS host-monotonic association, coordinate
  contracts, recording ownership, Legacy and device configuration are preserved.
- Accepted product risk: app receive includes per-side transport/decode latency. The user defines it
  as distance timing truth; provenance still exposes the boundary and NTP diagnostics for audit.

## Proposed indoor experiment impact — awaiting approval

- Camera/media owner: same side process/pipeline. Classification and latch policy change only under
  the explicit environment flag; strict default and Legacy remain unchanged.
- GUI: existing Camera/status text gains one bounded experimental state. No layout/navigation,
  command or persistent setting is added.
- Measurement: app-time pairing and geometry are unchanged. Experimental media provenance flows
  with frame metadata so results cannot masquerade as strict-IDR input.
- Recording: same direct-copy owner and async validator. File growth may increase disk use by tens
  of GiB over one hour; approximately 1 TiB is currently free. Integrity rules do not relax.
- CPU/GPU: no new encoder and no new product worker. NVDEC/YOLO demand is unchanged; the external
  monitor samples rather than polling product queues. Acceptance uses decoded/inferred throughput,
  GUI latency, queue/drop and memory trends, not one utilization snapshot.
- Devices: no firmware, encoder, transport, PX4 parameter, reboot or SDK mutation. Process-local
  NTRIP credentials are discarded at exit and secure-store preference remains false.
- Rollback: unset the environment flag and restart. No data migration; experimental rows/files keep
  their provenance and are never rewritten.
- Residual risk: NAL1 is not a standards-guaranteed refresh point and an undetected source defect may
  escape. Real motion review and relatch evidence are mandatory; direct MP4 remains failed.

## Approved manual-only PX4 PARAM impact

- Owner: the existing explicit Pixhawk parameter command/worker remains the sole owner; only its
  implicit LIVE-status caller and connection-session marker are removed.
- UI: no new control, text format, geometry, focus, selection, or scroll behavior. Existing waiting,
  busy, success, and failure rows remain.
- Devices/resources: normal qualification sends 124 fewer PARAM requests per side and avoids an
  unsolicited command reservation/requalification. Manual demand and resource bounds are unchanged.
- Data: no schema or migration. Recording omits CAL_MAG slot snapshots until a manual readback has
  populated the cache, rather than triggering hidden I/O.
- Isolation: ATTITUDE/GNSS, RTCM, Measurement, cameras, recording, the peer side, and shutdown remain
  independent. Historical artifacts are not rewritten.
- Rollback: restore the isolated LIVE-status hook and marker; no device or data rollback exists.

## Approved 60-second MAVLink qualification impact

- Owner/state: only the existing per-side liveness reducer threshold changes; connection identity,
  heartbeat/attitude freshness, stall detection and explicit command ownership remain unchanged.
- UI: existing state/detail regions render `01:00` dynamically. No control, geometry, focus,
  selection, scroll, dock or splitter change.
- Measurement/RTCM: a healthy side becomes operational sooner; stale/stall fail-close remains
  immediate. Method geometry and attitude values are not transformed or reinterpreted.
- Persistence/deployment: no setting, schema, migration, dependency or device configuration.
- Evidence: existing ten-minute artifacts stay historical; new long runs remain separate from the
  one-minute product gate.

## Approved Mountain GNSS 3D-fix impact

- Owner: only `OpticalPoseController` Mountain translation observability changes; global live-GNSS
  readiness and the other seven Measurement methods keep their contracts.
- Inputs: both same-burst positions remain finite, the baseline remains finite/non-zero, and both
  numeric fixes must be at least 3. RTK label/accuracy/time remain evidence rather than blockers.
- Quality: non-fixed usable input adds an explicit warning, so comparison output is `ESTIMATED`.
  Sub-3D input leaves translation unobservable; rotation can independently remain blocked.
- Persistence: candidate quality adds numeric fix evidence inside existing structures. No schema
  version, setting, migration or historical-row rewrite.
- UI/resources/devices: existing text only; no control/layout, worker, queue, CPU/GPU owner,
  network command, parameter write or camera/PX4 mutation.
- Rollback: restore the isolated both-`RTK_FIXED` predicate and remove only the new evidence/warning.

## Mountain automatic-suggestion copy impact — approved 2026-08-29

- Workflow/UI: the existing explicit copy click remains; its Mountain enablement uses structural
  readiness instead of detector quality. Existing group geometry and Manual Mountain remain.
  Horizon is unchanged.
- Controller/domain: paired exclusion, identity, bounds, count, uniqueness and distribution are
  owner-validated before filtered points enter the annotation buffer. No widget becomes geometry
  truth and Solve receives no new input path.
- Safety: confidence no longer blocks Mountain copy, but stale identity and invalid point structure
  remain hard. Exact K,D, GNSS and robust solver observability remain later Solve gates.
- Persistence: additive copy-eligibility, exclusion/count/hash provenance only; no schema migration
  or old-row rewrite.
- Resources/devices: no process, thread, queue, CPU/GPU budget, network call or device owner change.
- Rollback: restore Mountain GOOD-only button/controller acceptance; retained additive evidence
  remains readable.

## Horizon automatic-suggestion copy impact — approved 2026-08-29

- Workflow/UI: the existing explicit copy click remains; Horizon enablement changes from detector
  quality to current endpoint structure. Quality text and Manual Horizon remain.
- Controller/domain: current identity/dimensions, exact two-per-side count, native bounds and
  25%-width separation are revalidated before endpoints enter annotations. Solve gets no new path.
- Persistence: additive `copy_eligible/copy_block_reason`, validated-input hash and low-quality
  warning only; no schema migration or old-row rewrite.
- Resources/devices: no worker, queue, CPU/GPU, network or device change.
- Rollback: restore Horizon GOOD-only button/controller acceptance; additive evidence remains
  readable.

## Approved Settings-preferred distance timing impact — 2026-08-31

- Settings/UI: the existing read-only basis field becomes selectable; default remains app receive.
- Pairing/state: one policy owner and generation prevent coordinator/freeze split-brain. Basis
  changes flush bounded buffers and do not add a worker, queue or device owner.
- Failure isolation: selected non-app timing failure falls back to app receive; Camera, STUS,
  recording, PX4/GNSS and geometry continue independently.
- Persistence: requested/effective basis and fallback reason are additive; exact app and NTP fields
  remain separate and historical rows are immutable.
- Compatibility: old `camera_rtc_required` values normalize to `camera_rtcp`
  preferred-with-fallback; missing/invalid values remain app receive. Existing rows are not changed.
- Resources: pure policy/state and additive fields have negligible CPU/GPU impact.
- Rollback: restore fixed app receive while continuing to read additive fields; never reinterpret
  rows already written under another effective basis.

## Proposed Mountain lever-arm status impact — interviewing 2026-09-01

- Coordinate contract: GNSS input remains P2-minus-P1 world NED metres; the optional correction
  remains Left camera optical RDF `[Right, Down, Forward]` metres; stored OpenCV stereo `R,t` and
  baseline-center triangulation conventions must not be reinterpreted.
- Narrow assumed-zero option: no numeric geometry change, worker, device command, setting, schema
  migration, or historical-row rewrite. Candidate provenance/UI warning text and warning-derived
  `VALID|ESTIMATED` behavior must be regression-tested because warnings participate in quality.
- Measured-calibration option: adds operator input, persistent per-rig values, validation and
  frame-aware conversion. This is a larger settings/controller/persistence/UI contract and needs
  measured values plus separate approval before design or implementation.
- Preserved: exact K,D, same-order Mountain correspondences, robust rotation gates, capture-bound
  GNSS baseline/fix gate, Left anchor dependency, Draft freeze, `pixhawk_rtk_live` representative
  result, append-only existing evidence, and all non-Mountain methods.
- Safety: silently deleting the warning while leaving provenance unnamed is not recommended;
  either retain an explicit assumed-zero state or apply traceable calibrated values.
