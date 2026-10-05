# Abnormal restart and native shared-memory recovery — 2026-08-27

## Scope and oracle

- Disposable data root: `/tmp/workstation-crash-restart-20260827.4LrbVC`.
- The application was launched with the canonical repository interpreter, Python user-site disabled,
  and that isolated `SASL_WORKSTATION_DATA_ROOT`.
- Acceptance required an active Measurement request before an exact application-PID `SIGKILL`, no
  surviving camera child or established camera socket, a clean restart that did not adopt the old
  incomplete run, an enabled `측정 시작`, a distinct new run ID, and verified cleanup after normal
  Quit.
- Camera decode/MP4 success was not part of this lifecycle oracle. Both FW 2.18.72 cameras still
  supplied zero NAL5 and therefore remained truthfully blocked by the independent integrity gate.

## RED evidence

- The first killed application owned Measurement
  `run-2c04703de1e44eefab53d3784b4ee2fe`. Its camera children and sockets exited, but
  `/dev/shm/psm_28caf0d8` and `/dev/shm/psm_fa82ab4a` remained more than 20 seconds; each was
  44,236,800 bytes.
- Removing the camera attachment's `resource_tracker.unregister()` made the isolated tracker test
  pass, but a second real killed application, Measurement
  `run-30ee9b21cf994e8dbc8e9bc7c60b72d6`, still left generic `psm_cac0d37e` and
  `psm_d0d54baf` segments of the same size more than 20 seconds.
- Only those four experiment-owned exact names were unlinked after their evidence was captured. No
  wildcard or arbitrary `psm_*` cleanup was performed.

## Implemented recovery boundary

- Every Workstation native frame ring now has an owner-identifying name of the form
  `sasl_ws_native_<owner-pid>_<16-hex-token>`.
- Spawned camera attachments retain the owning process's shared tracker registration. Graceful
  shutdown still closes and unlinks explicitly; tracker EOF can reclaim the rings after `SIGKILL`.
- Before Qt imports or UI construction, startup scans only the strict Workstation prefix. It unlinks
  an entry only when the embedded owner PID is dead, the entry is a regular file, and its UID equals
  the current user's UID. A live/current PID, a malformed name, a different prefix, or an entry that
  cannot be safely inspected is skipped.

## GREEN field evidence

### Real active-Measurement `SIGKILL`

- Application PID: `1093884`.
- Active Measurement: `run-3dba2f926b194cdcb6bb9af56cef1098`.
- Camera children before kill: `1094334`, `1094335`; shared tracker: `1094331`.
- Exact rings before kill:
  - `sasl_ws_native_1093884_92137fa1dd0c8595`, 44,236,800 bytes.
  - `sasl_ws_native_1093884_c1e0443a6fad2510`, 44,236,800 bytes.
- Two seconds after the start request, `SIGKILL` was sent to PID `1093884`. Five seconds later the
  application and both camera children were absent, both exact rings were absent, and there was no
  Python-owned established `10.0.0.231/.232:8554` socket. Only bounded TCP `TIME-WAIT` entries
  remained.
- The tracker reported its expected abnormal-exit diagnostic for 54 semaphore registrations and
  two shared-memory registrations, then reclaimed them. This warning is evidence of the forced
  termination, not a graceful-path warning or a surviving resource.

### Tracker-loss fallback and clean restart

- A controlled same-UID tracker-less segment named
  `sasl_ws_native_1095036_0765a08cf547992d` was created after its owner helper exited. It existed at
  4,096 bytes immediately before restart.
- PID `1095041` logged
  `[startup] reclaimed 1 orphan native-frame shared-memory segment(s)`; the exact entry was absent
  afterward.
- The canonical event file contained 26 rows both before and after this clean launch, proving that
  startup did not adopt or manufacture a run for the prior incomplete Measurement. The captured UI
  showed `MEAS WAITING`, `REC IDLE`, and `측정 시작` enabled.
- A second clean restart exposed the visible `측정 시작` accessibility node as
  `enabled=True`, `sensitive=True`, action `Press`. Pressing it created
  `run-46b99906e4574db09c07b4efe3ac92e4`, distinct from the killed run, and appended fresh
  `measurement_start_requested`, video-start, and recording-start evidence.
- A normal window close then exited the application with no Workstation child process, native-frame
  segment, established camera socket, traceback, Qt callback exception, or resource-tracker warning.
  Its terminal `shutdown_resource_verification` record has `complete=true`, empty live process,
  running thread, open socket, and shared-memory arrays.

## Automated regression

- Focused native-ring, evidence-writer, camera-recovery, and lifecycle selection:
  `84 passed in 5.22s`.
- The native-ring tests independently prove that the sweep removes a dead strict-prefix segment,
  preserves a current/live-owner segment and an unrelated shared-memory name, and that a killed
  owner helper's ring is tracker-reclaimed.
- Full Workstation suite including this slice: `1453 passed, 1 skipped in 122.41s`.

## Artifact hashes

- `run4.log`: `1bbfa4da16a3572b6e514a37380ab3b22f7dda02e5eb966088d2945975b4f346`
- `run5.log`: `24b12a07038e584aeb1a4552f6841d403e7510dfb2cc725eeed27c64b4a472bf`
- `run6.log`: `c032b883125f6b8e192c74498157e2b893fb561d7a8eb8b714a6c2a2071452b5`
- `run5-clean-start.png`: `0cb9e2fe3442125b875816b94f24e69929ca05efce4e85902332389dda4182b2`
- Final 35-row `events.jsonl`:
  `3395f461639008647c257663b82932ec33e4c7f8d965a70c3f4f997c0aaaa480`
