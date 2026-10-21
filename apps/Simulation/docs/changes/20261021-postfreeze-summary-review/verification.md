# Verification ledger: post-freeze results review

## Requested outcome and scope

Review and correct the interpretation of the v1.2 summary using existing files only. Keep the original summary workbook and frozen analysis inputs unchanged. Produce a separate post-freeze review workbook, make justified documentation corrections, and record what remains unverified. Do not run SITL or claim MC trials are independent physical flights.

## Completion criteria

- S0: Rehash the selected result inputs and record which large manifests were not fully rehashed; establish the frozen analysis ID and source/code scope.
- S1: Reaggregate available per-run/per-MC V0 records and label the unit and uncertainty correctly.
- S2: Reconcile tracking P95 and frame-level GT exclusions without rewriting original QC records.
- S3: Export a separate review workbook with source/formula trace, readable sheets, and independent arithmetic spot checks; leave the original summary untouched.
- S4: Correct the Huber print note and document result version/scope distinctions supported by the frozen artifacts.
- S5: Reopen/inspect the exported workbook and changed HTML, check scope preservation, and record portability/status evidence.

## Governing evidence and contracts

- Frozen analysis bundle: `apps/Simulation/recordings/stereo_v1_1/paper1_v1_2_001/analysis_versions/analysis_9cfd7c6cd10b/` (`analysis_id=analysis_9cfd7c6cd10b`).
- Original summary: `apps/Simulation/recordings/stereo_v1_1/paper1_v1_2_001/stereo_v1_2_summary.xlsx`; preserve byte-for-byte.
- Tracking P95 threshold: `apps/Simulation/작업지시서/stereo_calibration_39run_package_v1_2/09_작업지시서_상세설명.html`.
- Robust loss scope: frozen `analysis_freeze.json` and the frozen algorithm identity, checked against the explainer and source snapshot.
- Coordinate review: evaluation RMSE axis fields are `lateral_east`, `forward_north`, `vertical_up` in world ENU metres. QC review joins and reports stored values only; it performs no frame conversion or axis remapping. GT frame alignment rows remain in the producer's recorded frame contract.

## Slice map and status

- S0 — source integrity and scope: IN_PROGRESS. Inputs: freeze, source hashes, frozen CSVs, recording/QC evidence. Output: selected-hash verification and explicit unverified large-manifest boundary.
- S1 — per-run/per-MC V0 reaggregation: NOT_RUN. Depends on S0. Output: V0 grouping table and MC empirical summaries, kept distinct from flight-level inference.
- S2 — post-freeze QC reconciliation: NOT_RUN. Depends on source QC/GT evidence. Output: one row per accepted source run, P95 > 1 m flag, all-capture invalid GT count, science-interval invalid count, and source row reference.
- S3 — companion workbook: NOT_RUN. Depends on S1/S2. Output: one new XLSX; original XLSX unchanged. Fixed code-derived aggregates must be labeled; simple threshold logic should remain a spreadsheet formula.
- S4 — documentation/scope correction: NOT_RUN. Output: corrected Huber note plus scope/version clarification; no analysis rerun.
- S5 — end-to-end artifact verification: NOT_RUN. Output: export/reopen checks, visual review, diffs, and `git status` review.

## Initial observations

- Earlier visual inspection of the source workbook succeeded for the summary, execution, tracking, observation-quality, provenance, definitions, and bounded calibration/graph ranges. Rendering the entire used range of `04_교정비교` failed due to its large dimensions; bounded reference renders succeeded. Preserve this failed observation and use bounded render ranges.
- Existing source files expose per-case evaluation CSVs and per-frame `gt_at_images.jsonl`; initial inspection indicates available material for reaggregation without rerunning calibration or simulation. This is provisional until S0 hashes and coverage are checked.
- The 2,928,820-line checksum manifest is approximately 482 MB. Do not claim it has been exhaustively rehashed unless a bounded verification proves it.
- A dirty worktree predates this task and contains unrelated Simulation/Workstation edits. Only this change packet, the explainer note, and the new review output are in scope; preserve all unrelated changes.

## Change complexity

- KEEP: a separate, formula-traceable review workbook and a concise ledger; they make the review auditable without mutating historical records.
- DEFER: full corpus checksum verification and new SITL runs. Revisit if a paper claim requires independent stochastic flight trials or if a selected source hash fails.
- Architecture refresh: NOT_REQUESTED. No architecture owner/dependency change; this is an offline reporting and documentation correction.

## Slice updates

### S0 — source integrity and scope

- Status: PASS for selected sources; whole-corpus verification remains DEFERRED.
- Evidence: all 10 selected analysis bundle entries (freeze, methodology, run manifest, summary, path selection, calibration/evaluation metrics, GPS metrics, and attitude comparison CSVs) matched `analysis_checksums.sha256`. All three campaign-root manifest entries matched. For the 39 selected run attempts, `qc_report.json`, `gt_alignment_summary.json`, `events.jsonl`, `image_index.jsonl`, and `flight_reference.json` matched their attempt checksum manifests (195/195). `analysis_id=analysis_9cfd7c6cd10b`; freeze SHA-256 `3de51417136ffafd1bcdd4592deaf0ae44bf0f43148230eb901e271f92719de5`; `calibration_solver.py` SHA-256 matches the frozen algorithm identity `b672063887927d780242e4c9855cce542a6dc9b1a53f9b60718d5d2f0c350d0b`.
- Scope: selected input CSVs comprise 398,200 existing-yaw branch rows and 68,200 GPS_YAW90 rows. The full 2,928,820-entry / ~482 MB analysis checksum manifest and YOLO weights were not exhaustively rehashed. Per-frame `gt_at_images.jsonl` files are not covered by the source attempt checksum manifests; their individual SHA-256 values will be recorded in the review workbook.
- Tooling note: system Python lacks `openpyxl`; workbook extraction used the approved Artifact Tool read-only import instead. The initial attempt to start a command in a not-yet-created temporary working directory failed before execution; it had no repository effect.

### S1 — V0 MC reaggregation

- Status: IN_PROGRESS.
- Evidence so far: `evaluation_metrics.csv` has 398,200 rows / 3,982 condition groups; `gps_yaw90_evaluation_metrics.csv` has 68,200 rows / 682 groups. Every group has exactly 100 unique `trial_index` values, all rows are `EVALUATED`, and the eight V0 evaluation IDs are present in both families. Final group statistics and path-level spot checks are pending workbook generation.
- Unit/limitation: metrics are per-case outputs of 100 frozen MC trials; they are not 100 independent flights. V0 uses frozen camera poses and is not a camera-pose recalibration experiment.

### S2 — post-freeze QC and GT frame reconciliation

- Status: PASS for source reconciliation.
- Evidence: 39 accepted run slots mapped to their selected attempt. 34/39 source QC reports contain `some_images_lack_valid_gt`. Their 39 `gt_at_images.jsonl` files match the `gt_alignment_summary.json` image count and per-status counts. Across the 39 files, 9,788 image rows have invalid GT alignment; every one is before the logged `science_start`, with 0 invalid rows between each run's logged `science_start` and `science_end`. All rows report `gazebo_world_enu` position with `east,north,up` axis order. The per-frame files have no existing checksum manifest; their file hashes will be embedded in the new workbook.
- Tracking: source workbook `02_경로추종!M6:M44` contains 39 numeric P95 values, all > 1 m. The pre-freeze QC reports `tracking_position_p95_above_configured_limit` for 31 runs; eight V0 runs omit it because V0 tracking metrics were hidden in the pre-freeze QC. The review output will add a separate post-freeze flag and will not rewrite the historical QC JSON.

### S3 — first builder attempts (preserved failures; no XLSX written)

- Attempt 1: FAIL before workbook mutation/output. The review builder referenced an undefined local `p95` while assembling the QC row. Corrected to the inspected source value `tracking.p95`.
- Attempt 2: FAIL before workbook mutation/output. The checker incorrectly required every invalid image frame to have a stereo `pair_id`. Readback of `image_index.jsonl` showed the frame IDs exist but some leading unpaired images have `pair_id=null`. Corrected the contract: every invalid frame must have a corresponding image-index row; pair ID is included when present and blank when the image was not paired. The workbook will report the count of these unpaired invalid frames.
- Both failures occurred before sheets were added or an XLSX was exported. The source summary workbook and selected analysis inputs were not changed. The spreadsheet authoring marker was run once before the first builder attempt; retries will use the same authorized output path without repeating the marker.
- Attempt 3: INTERRUPTED after 270 seconds with no workbook or preview file present. The builder emitted no stage progress, so the delay boundary was unknown; no repository source data or original workbook was modified. The builder will be instrumented with checkpoints before another attempt. The spreadsheet authoring marker remains one-time for this output.
- Attempt 4: FAIL at the pre-export scope-preservation check after the in-memory workbook recalculated. It flagged the blank `05_그래프!A16` as changed; a read-only source read and standalone recalculation both show the cell is blank with no formula. The comparator treated blank representations as unequal. The next check normalizes only `null`/undefined/empty-string cells as equivalent and still rejects any nonblank source value or formula change. No XLSX was exported.
- Correction to Attempt 4: an instrumented rerun showed the actual difference was not blank-cell normalization. The graph title had been written into `05_그래프!A16`; the original headings are at `A18`, `A91`, and `A182`. The earlier blank-cell explanation is superseded. The builder now asserts those observed cells and writes only those three titles. No XLSX was exported in either attempt.
- Correction to the title-cell mapping: the second comparison confirmed the actual paired-count title is `05_그래프!A17` (the earlier range-index note incorrectly said A18). The verified destinations are now A17, A91, and A182. The scope guard again prevented export; the source workbook remains untouched.
- Correction to the correction above: a direct Artifact Tool read of `05_그래프!A16:A19` shows the paired-count title at A18 (A16 and A17 are blank). A17-only preflight failed before any sheet writes or export. Final target coordinates are A18, A91, and A182, each checked by direct cell read before editing.
- Correction to the title-cell failure interpretation: the builder wrote the intended title to A18, but the source graph used range begins at A2 (`A2:G222`). The preservation checker incorrectly labeled row index 16 as A17 because it assumed a row-1 origin. Direct cell read confirms A18 is the original title; the comparator now derives its starting row/column from the used-range address. Earlier A16/A17 target hypotheses are superseded.

### S3 — first exported workbook and visual review

- Status: FAIL pending one display correction; the XLSX was created and re-imported. The workbook has 13 sheets and 22 charts (21 original + the new GPS paired chart). Source cells, formula ranges, chart counts, and the summary formulas passed the builder's pre-export/round-trip guards. Visual rendering found `#NAME?` in the Formula Map because literal formula examples beginning with `=` were treated as live formulas. The builder will store those examples as text and the existing output will be patched in place, then re-opened and checked again.
- Additional graph audit: the large C_no_drone RMSE values in the existing V0 trade-off table are not a cache/aggregation error. Direct aggregation of `evaluation_metrics.csv` reproduces G0 P0–P4 values exactly (including 69,151.8986 m for P0); the workbook has separate C diagnostic/log-scale graphs. No change to those source values is warranted.

### S1 — V0 MC reaggregation

- Status: PASS.
- Coverage: 398,200 existing-yaw branch rows + 68,200 GPS_YAW90 rows = 466,400 outputs; 3,982 + 682 = 4,664 condition groups. Every group has exactly 100 unique trials, all status `EVALUATED`, across the eight V0 evaluation run IDs. The frozen evaluation code identity is verified for both `analyze_campaign.py` and `calibration_solver.py`.
- Independent checks: the new G0/G1 route rollup matches the original workbook's V0 path means in `05_그래프!B94:E99` within `1e-9 m` floating-point aggregation tolerance. A separate Python calculation from raw CSV for `G0-P2-R03 / G0-V0-R03 / Y0 / all_data / A_rtk_independent / RPY_IMU` gives n=100, RMSE mean 1.2144334696996024 m, sample SD 0.07582831177712335 m, median 1.1794113934978379 m, q2.5 1.1519456236735313 m, q97.5 1.4034986447755036 m, and mean P95 2.45503111804925 m. The XLSX cells match within 2.3e-16 m.
- Selection interpretation: P2 remains the pre-V0 predicted covariance winner (mean σ 0.611701 m vs P4 0.611774 m); no numeric pass threshold exists. In the descriptive G0 V0 `all_data` case means, P0 RMSE is 1.17184 m (A) / 1.17144 m (B), compared with P2 1.17993 m / 1.17969 m. These are not flight-level significance claims; run independence remains unverified.

### S2 — post-freeze QC / frame reconciliation

- Additional finding: 180 of the 9,788 invalid GT frames have a valid `image_index.jsonl` row but `pair_id=null`; they are unpaired images. Their `frame_id`, time, status, and pre-science phase remain traceable. No invalid GT frame falls within the logged science interval.

### S3 — companion workbook and graph scope correction

- Status: PASS. Output: `apps/Simulation/recordings/stereo_v1_1/paper1_v1_2_001/outputs/20261021-postfreeze-summary-review/stereo_v1_2_summary_postfreeze_reviewed.xlsx`; SHA-256 `17a2d78240ad99e91b707323b62d9169e0d7d51d0fdf357467638d8c37f032bf`; size 1,720,860 bytes. The original summary workbook remains at SHA-256 `2b425283607ef8a5c341da80c1173e963f6d52f64ef0c8d4f605e7dbd4c60e99`.
- The companion retains the original eight sheets and adds five review sheets: `06_리뷰판정`, `07_사후QC`, `08_V0_MC분포`, `09_GT제외프레임`, `10_계산추적`. Original cell values/formulas and chart data series are preserved; only the three identified scope-title cells and three corresponding chart titles were clarified. Chart count is 21 original + 1 GPS_YAW90 paired chart.
- Summary formulas recalculate to 39 P95 flags, 31 source P95 warnings, 34 source GT warnings, 9,788 invalid frames, 0 science-interval invalid frames, 4,664 MC groups, and 180 unpaired frames. Formula Map's two formula examples are stored as text; the final XLSX XML scan reports 0 error cells.
- Graph scope: existing RPY/YAW comparisons are `EXISTING_YAW` primary `all_data` and exclude GPS_YAW90. A separate GPS_YAW90 paired comparison chart now shows all_data and matched_200 by method. The existing P2 GPS diagnostic chart remains.
- Visual review: summary, QC, MC, invalid-frame, formula-map, original V0, existing paired-comparison, and new GPS paired-comparison renders were inspected. The detailed QC/MC/frame tabs are wide and require horizontal scrolling; their headers and rows are filterable.

### S4 — documentation correction

- Status: PASS. The explainer now says Huber applies only to image residuals and RTK/IMU sensor costs remain quadratic after whitening. Its Evaluation row no longer claims V0 pose-error re-estimation; V0 evaluates localization with camera poses frozen. The Oct 18 management snapshot and the MC progress census are labeled historical and linked to `analysis_9cfd7c6cd10b` and the companion workbook. All relative workbook links resolve.
- The visually large C_no_drone route RMSE values were checked against raw `evaluation_metrics.csv`; they match and remain a separate diagnostic rather than a spreadsheet error.

### S5 — final verification and repository status

- Status: PASS. The saved XLSX was re-imported with Artifact Tool, all 13 sheet names/order, 22 graph charts, original chart sources, summary formulas, and literal formula-map examples were verified. Independent CSV arithmetic spot-check passed.
- `python3 tools/check_portable_paths.py`: PASS. `git diff --check` for the three HTML files: PASS. No tests or new SITL runs were performed; this work reaggregated saved metrics and recording artifacts only.
- The original workbook and frozen analysis bundle were not modified. The new XLSX is under the Git-ignored `recordings/` tree. The builder and verification ledger, plus the three edited HTML files, currently appear untracked in Git status; no files were staged. Preview PNGs are disposable `/tmp` QA artifacts.
- Coordinate contract: no transform or axis remapping was applied. Evaluation positions remain world ENU `east,north,up` in metres; invalid GT frame IDs retain their `gazebo_world_enu` producer label and axis order. All 39 frame sets passed this contract check.

## Final superseding status

- S3: PASS. The Formula Map examples beginning with `=` were changed to literal text; the final values re-open as strings with no formulas and no `#NAME?`. Final workbook SHA-256: `17a2d78240ad99e91b707323b62d9169e0d7d51d0fdf357467638d8c37f032bf`.
- S4: PASS for the Huber correction, V0 frozen-pose scope, current analysis ID, and historical Oct 18 snapshot labels. Three relative workbook links resolve from their HTML pages.
- S5: PASS. The OOXML scan of the final saved XLSX found 0 error-typed cells. Its summary formula caches are 39, 31, 34, 9,788, 0, 4,664, 4,664, 466,400, and 180 for the corresponding metrics. The standalone exhaustive Artifact Tool cell scan was stopped after a prolonged read; its required checks were covered by the builder's re-import/round-trip checks, Formula Map-specific re-import, OOXML error-cell scan, and independent raw-CSV arithmetic spot check.
- Final original workbook SHA-256 remains `2b425283607ef8a5c341da80c1173e963f6d52f64ef0c8d4f605e7dbd4c60e99`. The full case-checksum manifest and YOLO weights remain outside exhaustive rehash scope.
