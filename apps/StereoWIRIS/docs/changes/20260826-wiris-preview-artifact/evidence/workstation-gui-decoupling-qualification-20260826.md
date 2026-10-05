# Workstation GUI-decoupling qualification — 2026-08-26

## Implemented boundary

- Preview: scaled, bounded latest-only queue.
- Detection/stats: independent bounded metadata queue.
- Stop: out-of-band multiprocessing Event; it does not traverse an image queue.
- Native raster: per-camera preallocated shared-memory ring with exact
  `(camera_id, stream_generation, frame_id)` identity and pin/lease protection.
- Accepted evidence: exact left/right slots only, asynchronous atomic PNG/fsync/decode/hash writer,
  durable `SAVED` ACK, and lease release on every terminal path.
- Pairing and preview geometry/optical math: bounded off-GUI workers with latest-only results.
- GUI: preview/result/status snapshots only; no continuous native QImage signal in the normal path.
- CUDA inference: explicit 15 Hz limit instead of runtime `max_rate=0`.

## Automated evidence

- Full Workstation suite: `1262 passed, 1 skipped` in 112.15 seconds.
- Focused GUI/native/stream/measurement suite before the full run: `192 passed`.
- Synthetic qualification command:

  ```text
  PYTHONPATH=.. .venv/bin/python apps/Workstation/qualify_camera_backpressure.py \
    --duration 600 --gui-stall 2 --evidence-period 2
  ```

- Synthetic result: PASS.
  - duration: 600 s
  - deliberate consumer/GUI stall: 2 s
  - native frames: Left 8,999; Right 9,000
  - ring capacity: 16 per camera; dropped 0; active leases at end 0
  - queue high-water: preview 1, metadata 4, control 1
  - preview/metadata replacements were explicit; queue overflow drops were 0
  - accepted evidence: 296 pairs SAVED, failed 0, queue-full 0
  - one overwritten exact candidate was rejected explicitly; no nearest-frame substitution
  - Stop latency: Left 11.8 ms, Right 10.9 ms
  - parent RSS: 39,546,880 -> 56,033,280 bytes; high-water 56,033,280 bytes
  - unacknowledged lightweight result cache reached its declared bound of 128

## Real Workstation observations

- The pre-decoupling motion/person run grew parent RSS from roughly 0.5 GiB past 4 GiB and the
  desktop reported `__main__.py is not responding`.
- After native shared-memory integration, live preview runs held parent RSS near 396–521 MiB and
  camera child RSS near 313–318 MiB; `/dev/shm/psm_*` count matched active camera rings and returned
  to zero on normal shutdown.
- Left real-camera preview remained live through RTP gaps; examples recovered in 0.543–4.503 s in
  the same session without corrupt-output publication.
- Right camera currently accepts TCP on port 8554 but does not return an RTSP response. `GETH` and
  encoder readback remain `TRUE / 0 / 1 / 90 / 0`; one `SETH TRUE` reactivation and a 60-second
  zero-client cooldown did not recover it. NVIDIA and software clients both fail at the RTSP server
  response boundary. A right-camera reboot/power-cycle is outside the approved no-reboot scope and
  awaits operator approval.

## Remaining acceptance

- Dual-camera 60-minute soak cannot start until the Right RTSP service is externally recovered.
- After recovery, run Workstation from a fresh PID, exercise continuous person/hand motion and
  accepted evidence writes, verify RSS plateau/GUI heartbeat/queue metrics, then EOS-finalize and
  full-decode both MP4 files.
