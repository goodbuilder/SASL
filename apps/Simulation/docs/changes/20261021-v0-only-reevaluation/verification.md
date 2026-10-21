# Verification ledger: V0-only reevaluation

## Requirements

- V0-only reevaluation can reuse frozen MC100 calibration results without invoking calibration or sensitivity calibration.
- A new output directory contains only the matching calibration artifacts plus new V0 evidence and results.
- The parent freeze, parent calibration files, and raw N=600 pilot remain unchanged.
- The new V0 condition and its coordinate/data provenance are explicit; the development pilot is not promoted into the 39-slot campaign.

## Slice records

### S0 — baseline and contract

- Status: PASS.
- Evidence: parent freeze `analysis_9cfd7c6cd10b` declares 100 MC trials and P2 selection; current `analyze_campaign.py` hash matches the frozen hash. The N=600 pilot manifest marks it development-only and records the V0 reference unchanged, with spawn N=600 m, U=1.05 m, and a vertical-liftoff gate. Raw image/state bags and ENU ground truth exist; YOLO-derived observation files do not yet exist.
- Coordinate review: world ENU, PX4 local NED, camera optical RDF, and pixel `[u,v]` contracts identified from the producer and experiment code.

### S1 — isolated reuse and evaluation path

- Status: PASS.
- Evidence: the separate `reevaluate_v0.py` runner validates the parent freeze, algorithm/dependency identities, source attempt checksums, complete 8,000-batch grid, and each copied trial configuration against the frozen case checksum manifest. The runner invokes only the existing post-freeze V0 evaluator. Its focused tests confirm no calibration or sensitivity solver call and that parent calibration files remain unchanged.

### S2 — N=600 pilot data preparation and reevaluation

- Status: PASS.
- Evidence: YOLO processed 3,144 left and 3,140 right frames, with 1,399 and 1,402 detections respectively; 1,397 stereo pairs had detections on both sides. Reevaluation completed all 8,000 frozen calibration batches and wrote 88,000 evaluation result rows in 1,629.844 s. The run used CPU YOLO, reused the 100 saved camera-sensor draws, and reports `calibration_recomputed=false`, `sensitivity_calibration_recomputed=false`, and `counted_in_campaign=false`.
- Integrity: all 8,000 per-case evaluation result hashes and all 11 top-level checksums passed. The parent analysis freeze hash remains `3de51417136ffafd1bcdd4592deaf0ae44bf0f43148230eb901e271f92719de5`.
- Interpretation: the pilot completed its unchanged V0 science reference, but changed spawn north, spawn height, and entry gating together. It remains a development result and does not isolate the effect of N=600 alone.

### S3 — regression and portability

- Status: PASS.
- Evidence: `py_compile` passed; `test_stereo_v0_reevaluation.py` passed (5 tests); repository portability check passed. `git status --short` was reviewed from the repository root. The new source, documentation, dependency pins, and focused test are untracked and need normal Git staging; generated campaign recordings are ignored local data. Existing unrelated worktree changes were left untouched. No source flight or device access was part of this software path.
