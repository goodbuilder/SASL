# Manifest-driven sweep and workbook handoff

## Outcome and governing sources

Turn the default sweep entry into a manifest-driven campaign reader. The campaign XLSX may classify supported parameters as campaign-fixed or run-variable for each work instruction. When the instruction requires seeds, consume the manifest seed and record component-level effective-seed evidence. Produce the specified human-readable campaign summary workbook from validated run and analysis evidence.

Authoritative requirements:

- `작업지시서/02_Codex_작업지시문.txt`
- `작업지시서/03_experiment_spec.json`
- `작업지시서/04_run_manifest.json`
- source dataset `../../../SASL_39회_시뮬레이션_20261009/출력형식_설계.md`

The source dataset is read-only. Its current `run_status.json` and `qc_summary.json` report 39 planned slots, all `VALID_WITH_WARNINGS`, and zero technical failures. Its README reports 32 science-completed runs and seven retained V0 entry-performance failures. Reuse these attempts; do not add duplicate flights.

## Acceptance criteria

| ID | Input → expected result | Evidence |
| --- | --- | --- |
| IN-01 | New versioned campaign XLSX → explicit ordered RunManifest; no implicit factor cross-product | Workbook schema readback; compiled manifest compared with work instruction manifest |
| IN-02 | Parameter rows with `campaign_fixed` scope → one campaign value reused for every run; `run_variable` → one validated resolved value per manifest run | Parser plan shows value and source for each run; negative cases reject missing/duplicate/fixed overrides |
| IN-03 | `seed_policy=manifest_required` → per-run `seed_requested` consumed; unsupported RNG evidence remains explicit; retries follow instruction | Seed audit in manifest/attempt records and QC |
| CLI-01 | Normal `run_auto_attitude_sweep.sh` invocation → campaign controller; legacy OFAT requires explicit action | Shell dispatch/help and CLI plan with configured workbook |
| DATA-01 | Existing 39-run source → read-only QC and artifact/hash audit; retain all seven performance failures | Source `run_status.json`, `qc_summary.json`, `manifest_actual.json`, and required-file audit |
| OUT-01 | Validated raw summary and released analysis → output XLSX with exactly the eight sheets in `출력형식_설계.md` | Reopened workbook structure, value reconciliation, workbook rendering |
| OUT-02 | Output XLSX → clear data/provenance/definitions, no fabricated estimator values, no V0 leak before freeze | Independent comparison to source summary and frozen analysis records |
| DOC-01 | Changed commands and workbook contract → `apps/Simulation/README.md` updated | User-confirmed README path and diff |

## Slice log

| Slice | Status | Evidence / next action |
| --- | --- | --- |
| Baseline and scope | PASS | Read work instruction, experiment spec, manifest, output design; found existing read-only 39-run source and existing eight-sheet handoff workbook. Root source checksums: 365/365 pass. Current run status: 39/39 `VALID_WITH_WARNINGS`, zero technical failures, 32 science-completed, seven retained V0-entry performance failures. |
| Offline-analysis release | PASS | Existing local analysis freeze status is `FROZEN_BEFORE_V0_POSITION_EVALUATION`, says V0 error was not read before freeze, records 100 MC trials, selected path P1, and no V0 recalibration. Its 36,071 analysis checksum entries passed readback. The older source handoff XLSX still labels all estimators unimplemented, so its comparison sheet is stale relative to this frozen analysis. |
| v4 input schema and reader | PASS | `config/campaign/v1_1/sweep_manifest_v1_1.xlsx` has 11 ordered sheets, a 149-item supported catalog, 143 campaign-fixed settings, six run-variable settings and 234 per-run assignments. `Vehicle.model_id=x500_2m` and `Vehicle.reference_link=vehicle_reference_link` are explicit campaign-fixed values and cannot vary per run with the current runtime. Unmeasured vehicle dimensions and antenna lever arm remain null handoff requirements. Reader matched all 39 work-instruction rows, IDs, roles, splits, attempt IDs and seeds (`2026100501`–`2026100539`); plan requirements passed. Negative tests reject missing variable assignments, fixed-value overrides, duplicate seeds, an unsupported vehicle model and reference link. Legacy v2/v3 parser tests pass. |
| Default sweep dispatch | PASS | No-argument `./run_auto_attitude_sweep.sh` used `xlsx_plan`, returned all 39 rows and did not start SITL. `--help`, `--legacy --help`, and the RQT `--stereo-v1-1` compatibility route were verified. The old OFAT workflow is available only via `--legacy`. |
| Source audit and output workbook | PASS | Reused the read-only source; 365/365 source checksums and 36,071/36,071 frozen-analysis checksums passed. Reopened output XLSX has exactly the eight required sheets, 39 run rows, 12 comparison rows and both original native charts. Readback reconciles first/last run IDs and seeds, vehicle model/reference link, QC status, split, analysis metrics, freeze status, and provenance hashes. Rendered `00_요약`, `01_실행현황`, `04_교정비교`, `05_그래프`, `98_출처`, `99_정의` and representative input sheets. Input workbook SHA-256 `d8b0f112fae80bfffb01c5338b47b2645545ad1ba5e097f538206fadca442194`; handoff workbook SHA-256 `6f40357873e5ef2f605497a9723c8011d4d2ea97d25b3db849c0a1e7427454c6`. |
| Workflow verification | PASS | Focused campaign, workbook, QC, seed, control-config, status, legacy-sweep and RQT tests: 61 passed. `py_compile`, `bash -n run_auto.sh run_auto_attitude_sweep.sh`, and `git diff --check` passed. README and operator help describe the new default, workbook contract and explicit legacy action. No new Gazebo runs were started. |

## Coordinate and result-integrity contract

Keep world ENU `[east,north,up]`, PX4 local NED `[north,east,down]`, optical RDF `[right,down,forward]`, XYZW world-from-reference camera truth, WXYZ PX4 attitude, Gazebo simulation exposure timestamps, and separate receive/inference timestamps. Do not expose V0 positioning metrics before the frozen-analysis release condition is verified. Performance failures with intact evidence remain data; technical/protocol failures remain distinct.

## Follow-up audit finding

The work instruction also requires lighting/background and the existing wind condition to stay frozen and be logged. Live preflight records the `simulation.env` and G0/G1 profile hashes, but does not record the world SDF hash or a wind setting. The existing source's `environment_report.md` predates the completed runs and says 0/39 verified, so it cannot establish those conditions for this dataset. If a future campaign changes or compares environmental conditions, add an explicit `EnvironmentContract` and preflight evidence before execution; do not infer historical wind/light state from the current workspace.

`Vehicle.reference_link=vehicle_reference_link` is now an explicit fixed workbook parameter alongside `Vehicle.model_id`. Physical GNSS lever arm remains null because the source geometry contains no antenna mount evidence.

## Follow-up audit resolution (2026-10-11)

| Slice | Status | Evidence / remaining limit |
| --- | --- | --- |
| Environment contract | PASS | Manifest schema v2 adds `EnvironmentContract` with six campaign-fixed world, asset-hash, lighting/background, weather and wind rows. The checked-in source world and installed launch copy both match SHA-256 `7590a6b74ab5e34a2ea14abb7432ceb11151747d7edf1fa431674ab05512b511`. Preflight compares both, checks the world name and records the parsed sun/material/atmosphere snapshot; it blocks explicit weather or wind elements/plugins. The runner now sets and records `PX4_GZ_WORLD` from the contract. The existing 39-run source is still not retroactively certified for these settings. |
| Offline-analysis control boundary | PASS | `STEREO_V1_1_MC_TRIALS` is parsed only for fixed-protocol `resume`, `all_results` and `reprocess`. The `xlsx_*` actions ignore it; `OfflinePlan.trial_count` remains frozen handoff metadata and is printed by `xlsx_plan`. |
| Workbook and plan workflow | PASS | Reopened by the campaign parser as schema v2; 39 run rows, all original IDs/seeds, six run-variable settings and 234 assignments remain intact. `./run_auto_attitude_sweep.sh` reports `WORKBOOK_PLAN_VALIDATED`, `execution_enabled=false`, 39 rows, and the environment/offline plans; it did not start SITL. Workbook SHA-256 is `e40bf92183e130a1331e4279f05562cf024a0ec6f79a367baab86d22856a2347`. |
| Focused verification | PASS | `../../.venv/bin/python -m pytest -q tests/test_stereo_v1_1_environment_contract.py tests/test_stereo_v1_1_workbook_campaign.py tests/test_stereo_v1_1_control_config.py tests/test_stereo_v1_1_campaign_runner_plan.py` passed 44 tests. The environment-contract tests read both world files and intentionally reject a wrong hash or world name. |
| Live SITL preflight | NOT_RUN | The live preflight starts short Gazebo capture probes. The environment-contract validation was tested directly against the source and installed SDF copies; no simulation run or probe was started for this cleanup. |
| Frozen resume-preflight binding | PASS | A follow-up test verifies that `xlsx_resume` supplies `PX4_GZ_WORLD` from its frozen workbook, even if the currently selected workbook declares a different world. The focused suite rerun passed 45 tests after this check was added. |
| Attempt provenance handoff | PASS | The follow-up suite verifies that a workbook attempt manifest records `gazebo_world_name` and that `effective_config.json` references the SHA-256 of `environment_report.json`. Final focused command passed 46 tests. |
| `simulation.env` organization | PASS | Moved the literal `STEREO_V1_1_*` action block to the top, clarified which actions own `SWEEP_*`, mission defaults, YOLO defaults and `STEREO_V1_1_MC_TRIALS`, and preserved all 172 pre-existing assignments and values. No legacy workflow variables were removed. |
| Post-reorganization control verification | PASS | The control-config suite passed 21 tests. A fresh `./run_auto_attitude_sweep.sh` read the reorganized file, validated all 39 manifest rows, printed `mc_trials: null`, and ended with `execution_enabled=false`; no SITL process started. |
| Requested `xlsx_all` attempt | BLOCKED_BEFORE_RUNS | Used fresh output ID `manifest_env_verified_20261011`. Manifest/workbook and all six environment-contract checks passed, but static preflight failed `root_python` (`px4_msgs` was not on the inherited interpreter path) and `isolated_yolo_python_and_device` (Simulation `.venv` Torch 2.12/cu132 found no CUDA GPU). Full-campaign storage was not measured because static checks failed; current free space was 232 GB. No run slot started and no SITL process remained. |
| ROS overlay diagnosis | PASS | Sourcing the same Humble, workspace, `px4_msgs`, and Simulation overlays used by the RQT launcher makes the controller interpreter import all required packages. The isolated YOLO interpreter still cannot allocate CUDA; the matching model runtime is pinned to Ultralytics 8.4.127. The next action depends on the user's CPU-versus-GPU choice. |

## Requested CPU campaign follow-up (2026-10-11)

| Slice | Status | Evidence / next action |
| --- | --- | --- |
| Campaign-local CPU runtime | PASS | User selected CPU for this campaign. Only `recordings/stereo_v1_1/manifest_env_verified_20261011/simulation.env` changed to `YOLO_DEVICE=cpu` and `YOLO_TORCH_BACKEND=auto`; the checked-in `config/simulation.env`, model weights, Ultralytics pin, workbook SHA and 39-row manifest stayed fixed. |
| Static preflight and resource check | PASS | After sourcing ROS Humble and the workspace overlays, all static checks passed. Sandbox preflight first failed because ROS/Gazebo UDP sockets were denied (`Operation not permitted`); the authorized retry outside the sandbox passed live short capture, storage check and cleanup. |
| First workbook run | FAIL | `G0-P0-R01/a01` completed science, controller exit 0, and 1,996 left/right source frames with 1,996 exact pairs and zero unmatched frames. CPU YOLO produced both observation streams. QC then reported `PROTOCOL_DEVIATION` solely for `workbook_run_manifest_vehicle_model_id_mismatch` and `workbook_run_manifest_vehicle_reference_link_mismatch`; the workbook stores these values under `parameters.Vehicle`, while QC incorrectly looked for absent top-level plan keys. The runner stopped before starting any other run. |
| QC repair and same-attempt recovery | NOT_RUN | Correct the expected-value lookup, add an audited revalidation limited to those exact false-positive QC errors, preserve the original report, and resume the same frozen campaign without rerunning R01. Then verify 39 valid slots and all per-run artifacts. |

## CPU campaign execution readback (2026-10-12)

| Slice | Status | Evidence / remaining limit |
| --- | --- | --- |
| Workbook QC lookup correction | PASS | QC now compares `run_manifest.vehicle_model_id` and `vehicle_reference_link` with the nested workbook `parameters.Vehicle` values. Fresh QC on the unchanged `G0-P0-R01/a01` source returned `VALID_WITH_WARNINGS`, with no infrastructure or protocol errors. |
| Same-attempt recovery | PASS | Official `xlsx_resume` revalidated `G0-P0-R01/a01` without a second attempt. `qc_report_pre_revalidation.json` preserves the original false-positive result; `qc_revalidation.json`, rebuilt attempt checksums, updated runner/QC reports and `changes_log.jsonl` record the correction and validator SHA-256. |
| Live CPU campaign | PAUSED_AT_USER_REQUEST | User chose `YOLO_DEVICE=cpu`, `YOLO_TORCH_BACKEND=auto`, with pinned Ultralytics 8.4.127 and pinned weights. Resume preflight passed. The runner completed R01–R04 and stopped before R05. `run_status.json`: 39 planned, 4 `VALID_WITH_WARNINGS`, 35 `PLANNED`, zero `RUNNING`; all four slots have one attempt, and no SITL process remained. `campaign_run._audit_valid` reported `planned=39`, `valid_readback=4`. |
| Completed-run readback | PASS | R01–R04 seeds are 2026100501–2026100504. Pair counts are 1,996; 2,015; 1,994; 2,013, all with maximum pair delta 0 ns. YOLO inference errors are zero. Each QC retains warnings for unverified stochastic independence and tracking P95 above the configured threshold. R03/R04 retain unmatched source frames as reported in their image indexes. |
| Seed boundary | PASS | `RunManifest.seed_requested` is explicit for every slot (2026100501–2026100539); runner sends each value only through `SIM_GAZEBO_SEED` to `gz sim --seed`. `px4_sensor_rng_verified=false` and `independent_repetition_verified=false`; this evidence does not establish a combined PX4/Gazebo seed or independent PX4 stochastic trials. |
| Code and workflow verification | PASS | Full Simulation suite: `../../.venv/bin/python -m pytest -q` → 1,838 passed, 125 skipped. Focused QC/status/workbook suite → 28 passed. `py_compile` and scoped `git diff --check` passed. |
| Full 39-run acceptance | PAUSED | Only 4 of 39 manifest slots have QC results. The other 35 remain planned in the same output ID `manifest_env_verified_20261011`; resume with its frozen workbook when execution continues. |

## Campaign resume and second pause (2026-10-12)

| Slice | Status | Evidence / remaining limit |
| --- | --- | --- |
| Same-output resume preflight | PASS | Resumed output ID `manifest_env_verified_20261011` with its frozen workbook SHA-256 `e40bf92183e130a1331e4279f05562cf024a0ec6f79a367baab86d22856a2347`; CPU/auto runtime and Ultralytics 8.4.127 remained selected. Live short-capture and storage preflight passed. |
| Runs after resume | PASS | Completed manifest rows R05–R18 without creating a second attempt for R01–R04. All 14 new slots are `VALID_WITH_WARNINGS`. V0 rows R06, R11 and R16 did not reach science start; raw captures and controller failures are retained and QC classifies them as performance warnings. V0 tracking metrics remain unrevealed. |
| Second pause/readback | PASS | Stopped after R18 QC and before R19 as requested. `run_status.json`: 39 planned, 18 `VALID_WITH_WARNINGS`, 21 `PLANNED`, zero `RUNNING`; `manifest_actual.json` records 18 attempts. `_audit_valid` returned `planned=39`, `valid_readback=18`; no active SITL remained. `changes_log.jsonl` records the pause boundary. |
| Full 39-run acceptance | NOT_RUN | Paused at 18/39 by user request; 21 manifest slots remain. Continue later with `STEREO_V1_1_ACTION=xlsx_resume` and the same output ID. |

## v1.2 instruction / XLSX sufficiency audit (2026-10-17)

Scope: review the existing sweep for the new instruction package, retaining XLSX as
the operator's campaign input without requiring a separate input manifest JSON.
This entry does not authorize or claim resumed flights. No runtime code, workbook,
old recordings, or earlier verification entries were changed by this audit.
Source HEAD: `6810b228e7783c6420fd456e877c978e56a66872`, with pre-existing worktree
changes, including deletion of the old top-level instruction files. Those changes
were preserved. Historical PASS entries above are not v1.2 acceptance evidence.

Governing requirements: `apps/Simulation/작업지시서/stereo_calibration_39run_package_v1_2/`
(`02_Codex_작업지시문.txt` and `03_experiment_spec.json`). Inspected input:
`config/campaign/v1_1/sweep_manifest_v1_1.xlsx`. Paths below are relative to
`apps/Simulation`; campaign module names refer to `campaigns/stereo_v1_1/`.

Method: compare instruction requirements with raw XLSX rows, parameter resolution,
runtime consumers and validation code. Independently inspect capture/QC and offline
analysis. Run existing isolated tests and a pure low-rate counterexample; do not
start SITL or inspect/re-evaluate V0 positioning outcomes. A supported parameter
range or a PASS on a component test is not proof of campaign compliance.

| ID / required outcome | Result | XLSX / owner / evidence and remaining work |
| --- | --- | --- |
| INPUT / XLSX is sufficient as campaign input | FAIL | `workbook_campaign.py:296` additionally opens `작업지시서/04_run_manifest.json` and checks its hash. That file is absent in the current worktree; `build_workbook_plan(DEFAULT_WORKBOOK)` fails before producing a plan. Remove the external-input dependency while preserving semantic validation and output provenance; do not merely disable all checking. |
| MATRIX / exact 39 conditions | PASS, isolated comparison | Raw XLSX `RunManifest` matches package run IDs, seeds, groups, paths, repetitions and roles. There are 149 catalog parameters: 143 campaign-fixed and six run-variable (four Path fields and two optical headings). All 39 path equations/endpoints match the v1.2 spec. Raw extraction and isolated supported-run/visibility checks pass for 39 rows; this does NOT bypass or repair the failed production plan. |
| APPLY / per-run values reach execution | PASS for inspected code path; runtime NOT_RUN | `workbook_campaign.py:480` resolves fixed/per-run values, `:1148` generates 70 scenario environment entries; `campaign_run.py:357` saves reference/config, `:499` selects generated scenario, `:530` passes workbook parameters to the controller. `load_sim_config.sh` loads base env then scenario. Operator need not edit env between runs. |
| CLIP / common 1000 m rendering limit | FAIL | Current workbook `CameraModel.far_clip_m=700`; instruction requires 1000. Scenario mapping overrides the base env's 500, so modifying only the base env would not fix workbook execution. |
| POSE / physical optical pose and separate PX4 mount | PARTIAL | Optical G0/G1 headings and optical 20-degree pitch are represented. Workbook camera PX4 mount pitch is 8 degrees on both sides, versus instruction 20. Keep optical RDF orientation distinct from PX4/body attitude. |
| MOTION / continuous reference and timing evidence | PARTIAL | Path generation is 50 Hz; controller interpolates by simulation elapsed time (`flight_controller.py:398`). Workbook `Motion.setpoint_hz=20`; cadence uses wall-clock sleeps. This establishes continuous reference use, not measured 50 Hz simulation-time command updates. Send cadence, simulation cadence and RTF acceptance remain unverified. |
| ENTRY / start and end gates | PASS for inspected configuration/code; runtime NOT_RUN | Position <0.5 m, speed <0.3 m/s, heading <3 degrees, 5 s stable window, 120 s readiness timeout and 5 s post-hold are present and consumed by `flight_controller.py`. Entry timeout is separately 300 s; clarify its distinction from readiness waiting in the user-facing contract. |
| FRAMES / explicit position and orientation contracts | PASS in focused software checks; runtime NOT_RUN | `trajectory_contract.py` uses ENU [east,north,up] to local NED [north,east,down] with observed origin; solver uses optical RDF to world ENU. `state_recorder.py` separates optical/world xyzw and PX4 body-FRD-to-NED wxyz. No new runtime origin, axis, pose or heading readback was obtained. |
| RAW / images, clocks, truth and pose retained | PARTIAL | Lossless image/CameraInfo bags plus clock/GT/pose JSONL are implemented. The instruction's minimum bag topic list additionally requires clock/GT/pose in bag. Data are not wholly missing; storage equivalence or an explicit contract update is needed. `capture_streams.py:24`, `state_recorder.py:46`. |
| COVERAGE / detect required-stream outages | FAIL by static coverage audit | CameraInfo is subscribed but not required nonempty (`capture_streams.py:28`, `qc_attempt.py:214`). Science-interval image coverage/gaps are not gated: GT alignment validates existing images only (`qc_attempt.py:257`), so zero science images need not produce a GT error. Add negative-path checks for absent CameraInfo, science-only image outages, and expected intrinsics. |
| RATE / distinguish requested and measured camera rate | FAIL; pure counterexample reproduced | `bag_index.py` records observed source rate, but QC does not compare it with requested camera FPS. `_yolo_rate_errors` uses `min(requested, source_rate)`: requested=20, source=5, attempted=5 returns `[]`. This is a function-level counterexample, not a full corrupted-bag integration test. |
| GT / minimum rate and interpolation | PARTIAL | Minimum GT rate is used by QC; no-extrapolation and bracketing checks exist. `preferred_gt_rate_hz` is range-validated but no producer-rate application was found. A desired value is not an observed sampling guarantee. |
| OBSERVATIONS / misses, pairing and source frame identity | PASS for inspected code and existing tests | Raw unmatched frames, null bbox on misses, original-resolution observations, unique pairs and pair tolerance are supported. CaptureTiming/ImageObservation flags are fixed capability assertions, not arbitrary feature switches (`workbook_campaign.py:1294`). Negative coverage gaps above remain. |
| RECOVERY / retain attempts and distinguish performance | PARTIAL | First valid attempt protection, same-seed attempts, fresh QC/hash audit and performance warnings exist. Two consecutive technical attempts block automatically only when their failure reasons match (`campaign_status.py:171`); detailed instructions require review after two consecutive technical failures of a run. |
| REQUIREMENTS / executable acceptance rules | PARTIAL | Workbook has 12 requirement rows. `_evaluate_plan_requirements` (`workbook_campaign.py:948`) evaluates four plan requirements; non-plan rows remain PENDING in this evaluator. Separate QC implements some contracts, but a row marked block is not by itself end-to-end enforcement. |
| OFFLINE / XLSX controls calibration and evaluation | FAIL as a unified XLSX workflow | `OfflinePlan` is read and copied, not dispatched. `control_cli.py:189` returns after XLSX flight runner. Analysis reads bundled `protocol/03_experiment_spec.json` (`analyze_campaign.py:2102`); MC config excludes xlsx actions (`control_config.py:166`). Connect one explicit analysis contract before claiming XLSX changes control the whole experiment. |
| METHODS / A/B/C and angle variants | PASS for static implementation presence; numerical acceptance NOT_RUN | Existing solver supports fixed camera centers, RPY_IMU/YAW_ONLY, C input restrictions, G1 zero heading, and freeze-before-V0 evaluation. Bundled v1.1 and package v1.2 calibration_contract/reprocessing/split_policy match. Version label alone does not invalidate these algorithms. No V0 errors were opened or recomputed. |
| MC / report exact 100-trial coverage | UNRESOLVED scope mismatch | `_calibrate_all` (`analyze_campaign.py:2297`) covers all calibration slots/yaw cases/matched samples at trial 0, but later trials only G0/G1 P0/P4 repetition 1 with Y0/nominal-zero and all-data. Package says mc_repeats=100 without declaring this representative-only restriction. Make coverage explicit and align before claiming complete main-comparison MC coverage. |
| FINAL / original operator workflow | BLOCKED / NOT_RUN | Current plan is blocked by INPUT. No short SITL, representative four-slot verification, full 39-run capture, or new final analysis was executed in this audit. |

Fresh test evidence (repository-root `.venv/bin/python -m pytest -q ... --tb=short`):

- `apps/Simulation/tests/test_stereo_v1_1_{workbook_campaign,trajectory_contract,flight_controller_logic,capture_streams,campaign_status}.py`: 27 passed, 5 failed earlier in this same review; failures stop at missing external instruction manifest.
- `apps/Simulation/tests/test_stereo_v1_1_{environment_contract,campaign_setpoints,flight_gates,camera_frame_contract}.py`: 8 passed, 4 failed; all failures stop at the same missing external manifest.
- `apps/Simulation/tests/test_stereo_v1_1_{qc_attempt,state_recorder,bag_index}.py`: 10 passed. The identified FPS/coverage gaps demonstrate why these passing tests do not establish complete requirement coverage.
- Combined disjoint scope: 45 passed, 9 failed. Each invocation reports the existing Matplotlib Axes3D import warning. No test failures were repaired in this review.

Recommended implementation sequence: remove the extra manifest input dependency while keeping
XLSX structural and semantic checks; align fixed values with v1.2; close raw-stream and cadence
acceptance gaps; connect/document the offline analysis contract and MC scope; then validate plan,
negative-path tests, short SITL and the four representative slots. Save generated run lists and
effective configuration as output evidence; those outputs do not require a second operator input.

Coordinate contract checked: world ENU, PX4 local NED/body FRD, optical RDF and
quaternion orders reviewed; coordinate/setpoint/frame-gate component tests passed.
Runtime frame/pose/cadence checks remain NOT_RUN.

### XLSX-only change plus one sweep invocation: follow-up check

User question: can editing XLSX alone and invoking sweep once satisfy v1.2?
Result: not established and not sufficient with the current implementation.

- Fresh, in-memory diagnostic applied `CameraModel.far_clip_m=1000`, both
  `CameraPX4Mount.*_mount_pitch_deg=20`, and `Motion.setpoint_hz=50` to all 39
  raw workbook rows. `validate_supported_run`, reference generation, camera
  reference-point visibility and scenario environment readback passed for all
  39. This was not a saved workbook edit, production plan PASS, or SITL test.
- Unmodified production `build_workbook_plan(DEFAULT_WORKBOOK)` still raises
  `the v1.1 instruction run manifest is missing`. Changing `protocol_id` away
  from the literal v1.1 ID would skip `_verify_current_protocol_manifest`
  (`workbook_campaign.py:300`); that is skipped verification, not added v1.2
  validation, and was not used to claim acceptance.
- `xlsx_all` can sequentially execute the whole workbook in one invocation,
  but only after preflight; it stops at the first non-valid QC result
  (`workbook_campaign.py:1529`). The default action remains `xlsx_plan`.
  Technical failures may require preserved attempts and resume. One invocation
  is not evidence of 39 completed science flights.
- The instruction explicitly defines mandatory completion as raw capture,
  QC and handoff (`02_Codex_작업지시문.txt:295`). Even for that narrower scope,
  stream coverage, CameraInfo, measured cadence and storage-contract gaps in
  the preceding audit need code/verification work, not merely XLSX edits.
  OfflinePlan dispatch and MC scope are additional gaps if full analysis is
  included in the requested end-to-end outcome.
- No simulation was launched, V0 positioning evaluated, production workbook
  saved, or runtime code changed in this follow-up. Coordinate contracts and
  units remain those documented above; only independent numeric acceptance
  and mapping checks were added.

## v1.2 implementation and fresh execution evidence (2026-10-17)

User authorized the recommended raw-collection implementation and execution.
Ownership is unchanged: workbook module validates/resolves campaign inputs;
capture/index modules own raw messages; controller owns setpoint timing; QC owns
acceptance; campaign status owns immutable attempts. No new analysis method or
V0-error-driven selection was introduced. Existing user worktree edits are retained.

| Slice | Status | Fresh evidence / remaining limit |
| --- | --- | --- |
| XLSX-only input | PASS | Removed external instruction-manifest lookup. Code validates the supported v1.1/v1.2 39-slot geometry, IDs/seeds/splits; unknown protocol IDs fail. Source-manifest hash is provenance, not a reopening dependency. Prescribed four pilots run first without adding slots. |
| Workbook revision | PASS | Existing XLSX now declares v1.2; clip 1000 m, both camera PX4 mount pitches 20 degrees, reference and setpoint 50 Hz. RunManifest unchanged. Artifact Tool import/edit/export readback verified 39 rows; source styles byte-identical, views/widths/validations/filter/conditional-format XML preserved. Source references updated to v1.2 package. |
| Raw capture / QC | PASS in tests; pilot runtime pending | Added raw state bag for clock/GT/poses, CameraInfo nonempty and actual intrinsics/dimensions gates, per-epoch science coverage/FPS/gap checks. A one-frame endpoint allowance handles finite windows; 1.5 nominal periods is the explicit engineering jitter bound. No percentage FPS relaxation. Performance aborts assess their recorded prefix. |
| Send timing | PASS in tests; pilot runtime pending | New `setpoint_cadence.py` and timing JSONL distinguish monotonic host clock from simulation ns/epoch, duplicate observations and RTF. Completed science requires coverage/count/gap evidence at 50 Hz. No dynamics changes. |
| Attempt policy | PASS in tests | Two consecutive infrastructure failures now block even with different reasons. First valid attempt and original data remain protected. |
| Launch configuration | IMPLEMENTED | `config/campaign/paper1_v1_2.env` selects XLSX all, CUDA, and fresh output ID `paper1_v1_2_001`; base env remains plan-only. Configuration paths are repo-relative. The pinned PX4 v1.17.0 checkout was resolved by the existing machine-local runtime discovery; no firmware change. |
| Storage | PASS for preflight estimate | Estimate includes entry/readiness/landing/origin bounds instead of fixed 60 s overhead. First live probe projected 10.12 GB compressed data plus 10 GB reserve, with 224.25 GB available. Compression/scene complexity remains an estimate, per-attempt disk guard remains active. |
| Focused regression | PASS | Repo-root `.venv/bin/python -m pytest -q apps/Simulation/tests/test_stereo_v1_1_*.py apps/Simulation/tests/test_stereo_setpoint_cadence.py --tb=short`: 141 passed in 107.06 s, existing Matplotlib Axes3D warning. A preceding disjoint-invocation integration subset passed 43 tests; do not sum duplicate test coverage. |
| Plan workflow | PASS | `bash run_auto_attitude_sweep.sh` returns WORKBOOK_PLAN_VALIDATED, 39, execution disabled. Workbook SHA-256 `3973763fc31c6ad700e50e365ae5e2a81de4db95e601cb29beb273d9c74d2bca`. |
| Live no-flight probe | PASS | `recordings/stereo_v1_1/paper1_v1_2_001/preflight/short_capture_G0-P0-R01_a001`: 5 s raw capture, left/right 100/99 images, GT 50 Hz, no GT gaps >=40 ms, optical orientation error 0 degrees, optical-center errors <3e-15 m. Actual Gazebo launched and cleanly stopped; no arming in this probe. Environment report PREFLIGHT_PASS. |
| Portability | PASS | `python3 tools/check_portable_paths.py` and scoped `git diff --check` passed. Newly added helper/test/config files must accompany the existing modified source/XLSX when committed; no commit performed. |
| Full raw campaign | RUNNING | Started actual sweep with `bash run_auto_attitude_sweep.sh --config config/campaign/paper1_v1_2.env` after ROS/main overlay and repo Python activation. New 39-run completion is not yet claimed. |

The two new source files `campaigns/stereo_v1_1/setpoint_cadence.py` and
`config/campaign/paper1_v1_2.env`, plus `tests/test_stereo_setpoint_cadence.py`,
are required tracked additions for portability. Runtime results stay under the
ignored recordings tree; build caches and virtual environments are not deliverable
source. The v1.2 instruction package itself was already untracked at task start.

Coordinate contract checked: world ENU reference, PX4 local NED position/velocity,
optical RDF intrinsics and independent camera PX4 body attitude; focused tests and
live optical camera pose readback passed. Flight origin/cadence evidence pending.

### First v1.2 pilot: retained infrastructure failure

`G0-P0-R01/a01` completed science (controller exit 0), recorded images/state
normally, and postprocessed YOLO without inference errors. Both science image
streams contain 527 unique frames over 26.3 s (20.038 Hz), maximum image gap
52 ms, no missing intervals. GT observed rate is 50 Hz with zero >=40 ms gaps.
QC has no protocol errors and one infrastructure error:
`science_setpoint_cadence_below_v12_requirement`. Tracking P95 and unverified
stochastic independence remain warnings, not retry reasons. The serial campaign
stopped before its next slot: 0 valid, 1 infrastructure-failed, 38 planned.

Measured send cadence: host 50.0003 Hz, unique simulation 53.1307 Hz,
RTF 0.9091, max simulation gap 36 ms against the explicit 30 ms engineering
bound. This is a real FAIL under the implemented gate, not evidence of missing
raw imagery. Read-only diagnosis compared the independent clock recorder:
all /clock simulation increments were 4 ms; host receipt was bursty. Host send
gaps maxed at 24.806 ms (none >30 ms), while simulation gaps included 91 x 32 ms,
16 x 36 ms and 49 duplicate stamps. Evidence indicates wall-periodic sampling
aliasing bursty simulation progression, not simple wall-loop overruns. It does
not establish whether Gazebo stepping or ROS delivery causes the burstiness.

Next correction: dispatch reference updates from actual simulation-clock callbacks
with separate host keepalive/watchdog, preserving clock domains and unchanged
trajectory/gains; keepalive must not count as a new reference sample. Preserve
this attempt and repeat only after focused tests and renewed preflight, with
the same run ID/seed and a new attempt ID. Do not relax the acceptance bound.

Independent review also identified two negative-path gaps to close before resume:
state-bag science coverage cannot rely solely on separate JSONL coverage, and
cadence for a dynamics abort must validate its recorded science prefix instead of
skipping all timing checks. Existing captured data remain immutable.

### Clock-driven repair before a02

Implemented `science_dispatch.py`: actual `/clock` callbacks select the current
observed sample in anchored 20 ms slots; no synthetic catch-up samples. Depth-1
clock subscription avoids replaying queued stale ticks. A separate host watchdog
keeps the last command alive; `reference_update=false` messages are excluded from
unique-reference cadence. Callback removal drains active callbacks before command
resource/log closure. Clock callback receipt age is recorded, not interpreted as
PX4 actuation latency. Aborts preserve their last observed science snapshot.

Raw state bag deserialization now writes `raw_state_index.jsonl`. Independent QC
checks raw GT rate, image-time GT brackets and raw-clock science coverage. Cadence
is recomputed from raw send records and actual science/abort events, rather than
trusting a saved success boolean. Read-only a01 audit decoded 5,085 raw GT and
25,188 clock messages plus both camera diagnostics; its raw-state coverage passed
and its cadence failure was independently reproduced. No a01 artifact was changed.

Fresh integration command: repo-root `.venv/bin/python -m pytest -q
apps/Simulation/tests/test_stereo_v1_1_*.py
apps/Simulation/tests/test_stereo_setpoint_cadence.py
apps/Simulation/tests/test_stereo_science_dispatch.py --tb=short`:
**159 passed** in 86.25 s, existing Matplotlib Axes3D warning only.
Portable path check and scoped diff check passed. New dispatcher/test source must
be included when committing, alongside the earlier untracked additions.

Source snapshots and a scoped binary diff are saved in the campaign's
`source_evidence/clock_dispatch_revision`; `changes_log.jsonl` records the repair.
The workbook/IDs/seeds/physical settings remain unchanged. The same configured
sweep invocation was restarted for fresh preflight and the next immutable attempt.
There is still no completed 39-run or final-estimator claim.

Coordinate contract checked: ENU/NED/optical formulas unchanged and tested;
a01's actual origin gate passed (world start error 0.0143 m, local error 0.0195 m,
heading error 0.00747 degrees). Schema documentation was corrected to distinguish
the raw camera-link FLU-to-ENU quaternion from an optical RDF rotation, retaining
the previous schema mapping in source evidence and logging the metadata revision.

### a02 completion and cadence-validator false-positive diagnosis

The second flight also completed science/landing normally. Its source-clock
dispatcher emitted exactly 1,315 reference updates with no duplicate source stamps
and separate 263 keepalives; measured unique simulation rate is exactly 50 Hz,
RTF 0.9043, maximum callback receipt-to-send age 2.72849 ms. The existing 30 ms
maximum-gap hard gate nevertheless classified a02 as infrastructure-failed
because 13 intervals were 32 ms. Runner stopped before another slot.

Independent raw-timing review against the specification established that the
30 ms command-gap bound was an added engineering heuristic, not a v1.2 requirement.
The specification requires /clock-based 50 Hz references and reporting cadence/RTF.
Using an immutable science-start anchor and 20 ms bins: a01 covered 1,171 of 1,315
bins (144 missing; 274 extra sends in occupied bins); a02 covered all 1,315 exactly
once. Every a02 32 ms gap spans adjacent occupied bins with phase moving from
0 to 12 ms, not a missed reference interval. No V0 outcomes were inspected.

This exposes a validator false positive, not a reason to rerun a02 for better
performance. Correct the acceptance method to full anchored-reference-slot
coverage, retaining endpoint/epoch/backwards/unknown/range checks. Burst sends or
keepalives cannot fill missing bins. Continue reporting max gap and warn above the
old heuristic; never claim perfectly periodic 20 ms host sends or measured PX4
actuation latency. Synthetic boundary, missing-bin, duplicate and burst tests plus
independent a01-fail/a02-pass replay are required before revalidation.

Revalidation must retain original QC/runner/checksum evidence and raw artifacts,
record criterion version and source hashes, and update only derived status after
fresh complete QC passes. It must be restricted to this exact cadence-only false
positive; unrelated infrastructure failures and actual missing bins remain blocked.
This explicitly supersedes the earlier 30 ms hard gate while preserving the
instruction's 50 Hz reference requirement.

### Audited a02 revalidation and resume

Implemented `reference_slot_coverage_v2`: every full 20 ms interval anchored at
the actual science_start must contain a real reference update; the final partial
interval is optional. Unknown/out-of-range timestamps, epoch changes, backwards
time and missing full slots fail. Keepalives/duplicates cannot fill a hole.
Maximum gap remains evidence and >30 ms is a jitter warning. Fresh boundary and
negative-path integration tests: 47 passed (cadence, dispatch, QC, ROS bridge).

Read-only full a02 QC returned VALID_WITH_WARNINGS, no infrastructure/protocol
errors, warnings for jitter, unverified stochastic independence and tracking P95.
The narrow `revalidate_cadence_qc_regression` path additionally verifies original
checksums and allows only a latest v1.2 cadence-only failure with a fresh complete
slot-coverage result. It rejects unrelated errors, changed raw source, missing
slots and non-latest attempts. Its status regression suite: 12 passed; existing
Vehicle recovery behavior remains supported. Portable-path/diff checks pass.

Applied recovery under the campaign lock with no SITL active. a02 original QC,
runner result and checksums were snapshotted before replacing derived metadata;
raw/controller files stayed unchanged. `qc_revalidation.json` records the reason,
old report hashes, validator/evaluator hashes and time. Campaign counts now:
**1 valid slot, 38 planned; a01 remains a failed technical attempt**. The same
configured sweep command was restarted for fresh preflight and remaining pilots;
P0 is not flown again. This is revalidation, not a new flight or independent trial.

### Legacy landing-label limitation found during monitoring

Do not interpret `landing_confirmed` as independently observed ground contact.
For P0/a02, request-to-confirm UTC difference is only 4.679 ms and both events
reuse the same 118.000 s snapshot. More decisively, the independent GT stream
still reports ENU up=102.411 m at 123.320 s. `px4_auto_mission.wait_landed`
accepts an EXTENDED_SYS_STATE ON_GROUND packet without freshness/source-state
validation. An old queued packet is plausible, but its exact cause is not proven
without the MAVLink packet history. Earlier wording about normal landing is
superseded: only completed science, 5.016 s post-hold and normal recorder/operator
shutdown are substantiated.

v1.2 asks for 5 s post-science hold and safe SITL shutdown, not physical landing
as part of the scientific trajectory. Do not invalidate science solely because
the simulated aircraft is airborne when the owned SITL processes stop. Record
landing as unverified, retain the original event, and use image/state exit=0,
operator exit=130, flushed bags/hash QC and absence of residual processes for
shutdown evidence. Avoid changing active controller source mid-campaign. A
future confirmation implementation must use fresh, source-matched state and a
current snapshot; receipt time alone cannot establish packet freshness.

### Progress through six valid slots; startup GT failure and repair

P1, P2 and G1-P4 also completed science and passed source QC. V0-R01 did not reach
its science start within entry timeout; it has intact raw capture and is retained
as VALID_WITH_WARNINGS/performance outcome, not rerun. Thus six record-QC-valid
slots included five completed science trajectories and one V0 non-entry outcome.
Independent G1 pose readback matched `Rz(-heading) R0(20deg)` with rotation-element
errors <4.3e-10 and center errors <3e-15 m. G0-P4 ENU east reference/GT correlation
was 0.9874 with observed overshoot retained, supporting direction correctness
without claiming stereo accuracy. Evidence JSON files are under source_evidence.

P3-R01/a01 completed science and all 1,324 complete reference slots, but failed
`gt_rate_below_50hz`: state JSONL mean 48.2667 Hz over 127.5 s, 166 gaps >=40 ms,
max 80 ms. Every GT gap occurred during startup source times 10.1–18.5 s; science
was 97.884–124.376 s and its images/GT were valid. Raw state bag GT began at 30.5 s
and recorded exactly 50 Hz with no gaps. Images include earlier warmup frames from
8.252 s, so startup loss is retained and visible. This attempt remains failed;
the full-capture GT threshold was not relaxed or scoped away to force a PASS.

Added `capture_readiness.py`, invoked after raw-bag CAPTURE_READY and before
JSONL state capture/controller for v1.2. It observes source-clock/GT only (no
vehicle commands), requires at least 2 simulation seconds of unique fresh ENU GT,
the XLSX minimum GT rate and max gap, and restarts stability on reset/outage.
It has a bounded 60 host-second timeout and saves capture_readiness.json. Raw
warmup frames remain intact. Legacy launch order is preserved for older inputs.
Actual runner handoff tests prove that a failed readiness check prevents state/
controller start and cleans up owned bag/operator processes.

Also corrected future structured landing labels to `landing_status_received`
with `physical_landing_verified=false`; the legacy teardown behavior was retained.
Earlier raw events remain unchanged and are covered by the landing-status review.

An initial new-test collection failed due to missing standalone Simulation import
paths; the new tests were corrected to the repository's existing sys.path pattern.
Fresh full campaign-focused suite including readiness/startup tests:
**182 passed** in 100.08 s; existing Matplotlib warning only. Portable-path and
diff checks passed. Readiness helper and its two test files are additional required
source additions for a portable checkout. Restarted the same configured sweep for
fresh preflight and P3-R01/a02, with unchanged workbook/seed/trajectory/gains.

### Stopped at P3/a02; unresolved command timing boundary

The new readiness probe passed with 101 unique GT samples over 2.0 simulation
seconds, rate 50 Hz and max gap 20 ms. P3/a02 completed science; state capture
now reports exactly 50 Hz and zero gaps >=40 ms. The startup GT fix worked for
this attempt. However its cadence QC found one missing full reference interval
out of 1,323 (slot 398, source interval [90.304,90.324) s). Measured reference
rate is 49.96218 Hz; surrounding source sends are 90.284 and 90.328 s. This is
a genuine missing slot under the currently frozen strict interpretation, not
the earlier adjacent-bin jitter false positive. Raw data and failed QC remain.

Following the instruction's two-consecutive-technical-failure rule, no third
P3 attempt was launched. Current state: 6 record-QC-valid slots (5 completed
science trajectories and 1 V0 non-entry performance outcome), 1 failed slot,
32 planned/unexecuted slots, 9 actual attempts. All owned SITL processes are
stopped. Neither 39-run completion nor final positioning evaluation is claimed.

RCA facts: independent clock recording has a 73.051 ms host delivery stall,
then a burst of 4 ms simulation ticks. Keepalives continued. Adjacent successful
callbacks took only 2.89 and 1.44 ms to send. All five independent clock samples
inside the missing interval would pass the cache freshness/skew tests (GT age
4–10 ms, local age 2–8 ms, pair skew 2.259 ms, future offset <=16 ms). Depth-1
clock coalescing is plausible, but controller-local callback loss versus rejected
snapshot cannot be proven from successful-send logs. Do not remove GT gates or
change QoS merely on this inference. Minimal next diagnostic evidence is every
controller clock callback's source/receipt/lock times, snapshot acceptance/reason
and dispatch decision. Detailed RCA is `source_evidence/p3_two_failure_rca.json`.

A research-definition question is pending: should command 50 Hz mean all anchored
20 ms intervals must be occupied, or a nominal target with observed command-loss
performance warnings? The instruction specifies 50 Hz but no command-loss tolerance.
GT/image requirements are unchanged. Until that definition is supplied, keep the
strict gate and stopped state; no unanswered question implies approval to relax it.

Partial handoff: `요약_결과/partial_capture_summary_6_valid.json`. Generating it exposed
a legacy summary bug: workbook QC uses counted_in_campaign whereas the summary
required counted_in_39. Fixed authoritative-flag handling with legacy fallback;
9 focused summary/orchestration tests passed, including V0 blindness and rejection
of explicit false campaign inclusion. The 182-test core run above remains the
fresh integration baseline; do not add overlapping test counts. Concurrent offline
analysis/requirements/GPS-yaw worktree edits were observed outside this task's owned
changes and were preserved, not claimed as part of this implementation.

### Controller callback diagnosis (authorized follow-up)

User authorized the proposed instrumentation and a separate DEVELOPMENT diagnostic
run. Main campaign remains 6 valid slots; failed attempts and strict QC unchanged.
Method: buffered_controller_callback_trace. Purpose: distinguish controller-local
clock callback loss from snapshot rejection and lock/dispatch delay. Source time
is ROS simulation ns; receipt/lock times are host monotonic ns. Position contracts
remain world ENU [east,north,up] and PX4 local NED [north,east,down]; no coordinate
conversion changes. Cache owns validity predicates, ROS adapter owns receipt and
callback locking, dispatcher owns slot decision/lock timing, controller owns trace
lifetime/persistence.

D1 / Diagnostic behavior: optional buffered records, same validity/control/QoS,
explicit record loss count, exclusive output creation after producer shutdown.
Verification: rejection/acceptance and dispatch negative-path tests, bounded sink
and persistence tests. Result: NOT_RUN (implementation underway).
D2 / Actual workflow: same frozen XLSX P3 source and seed, separate 개발시험 path,
no main-slot promotion. Verify raw timing against controller-local clock and
snapshot/dispatch records; lack of recurrence cannot prove root cause or repair.
Result: NOT_RUN. No main P3/a03 authorized by this diagnostic decision.

D1 PASS: 50 bridge/cache/dispatcher and existing coordinate/setpoint checks passed;
13 sink/startup-isolation/runner/controller checks passed (overlap excluded from
any aggregate claim). Additional post-composition sink/controller check: 6 passed.
A first invocation named nonexistent test_stereo_v1_1_campaign_run.py and collected
no tests; corrected to the repository's campaign_runner_plan and startup tests.
Portable-path and diff checks passed. DEVELOPMENT QC explicitly sets both inclusion
flags false even when raw QC would otherwise pass. Main status and prior P3 raw
file SHA256 snapshot saved as source_evidence/timing_diagnostic_before.json.
D2 RUNNING: selected frozen-XLSX G0-P3-R01, same seed, existing successful preflight
report/workbook hash validated by runner; development runner a01. Root execution
session 96925 owns the only SITL. No campaign retry guard changed or bypassed for
a main attempt. Runtime environment derived with load_control_config and
control_cli._campaign_runtime_env from config/campaign/paper1_v1_2.env before
invoking campaign_run run --development --frozen-workbook.

D2 first runtime: controller and capture exited cleanly, complete trace 20,272
rows/zero drops/zero producer errors. Full1323/1323 slots; no snapshot rejection.
Original command-hole failure NOT REPRODUCED. Max reference source gap32ms is
adjacent-bin jitter, not a missing slot. However postprocess failed safely on
existing main 파생_재처리/G0-P3-R01/a01 directory: workbook development preserves
run_id but legacy postprocess did not namespace output. No file overwritten.
First development QC remains failed with lifecycle evidence, not rewritten.
Repair only development-derived path; add collision regression. One bounded
unchanged-conditions repeat is for intermittent diagnosis, not selection of a
favorable science result. Main status/prior raw hashes will be rechecked.

Development collision repair PASS: 7 sink/path-isolation/startup tests passed.
Main status and all prior P3 files:101 SHA256 checks unchanged after a01.
Source snapshot for bounded a02 repeat: source_evidence/timing_diagnostic_repeat_revision.
D2 repeat RUNNING, root session2044. No scheduling/control/QoS/QC change.
Independent read review found no material policy/correlation defect. Limitation:
legacy bridge.close does not assert shutdown return/thread join success; producer
stop summary currently assumes normal shutdown. a01 controller exited0 and trace
producer row/error counts normal; no evidence of a shutdown timeout. Need explicit
thread-alive readback in future completeness assertion; do not mutate active-run
source during a02. Initial seeded dispatch legitimately has no paired observer row.

D2 repeat flight completed:1323/1323 full slots, no snapshot rejection,20,205
trace records and zero dropped/producer errors. Observer4055 vs independent6614
source ticks in common range;2559 ticks absent at observer, source gap max24ms.
No command hole in either development flight; historical slot398 cause remains
unproven. Both trace analyses saved in source_evidence/timing_diagnostic_a0*_analysis.json.
No further flights planned within this diagnostic slice. Postprocessing a02 active.

After a02 flight/controller shutdown, corrected the independent review's
completeness limitation: adapter saves executor shutdown+thread-alive readback,
controller passes it to producer-stop record, summary_v2 requires successful
shutdown as well as loss/error-free accounting. Unit checks cover timeout/alive
branches. This metadata strengthening was NOT used by the two completed flights;
their original summary_v1 artifacts and frozen source snapshots are preserved.
No rerun justified solely for this readback field; runtime claim is limited to
observed clean controller exit in those two executions.

D1 final PASS:31 sink/shutdown/observer/cache/dispatcher/controller checks passed,
existing Axes3D warning only. Portable-path and diff checks PASS. New required
source additions timing_diagnostics.py and test_stereo_timing_diagnostics.py are
untracked and must accompany a future commit; prior new dispatcher/readiness files
also remain untracked. Raw diagnostics are intentionally ignored recordings. No
required new state outside repo tree, no new dependency, no git staging/commit.

D2 completed: a02 postprocess and strict QC VALID_WITH_WARNINGS; both inclusion
flags explicitly false and no main-slot promotion. a01 safe postprocess collision
failure preserved. All101 main-status/prior-P3 file hashes unchanged, main counts
6 valid /1 failed /32 planned, no active owned SITL. See
source_evidence/timing_diagnostic_final_integrity.json.
Outcome: instrumentation and separated DEVELOPMENT workflow verified; historical
command-loss mechanism remains NOT VERIFIED because neither flight reproduced
the missing full slot. Stronger evidence now localizes observed tick omissions to
the controller callback ingress path before accepted snapshot/dispatch. It does
not identify DDS replacement versus executor scheduling versus upstream delivery.
Do not report successful diagnostic flights as repair, weaken QC, or launch main
P3/a03 from these results. Next causal evidence, if pursued, is middleware/executor
arrival-vs-callback tracing with source-message identity; no further flights made.
Coordinate review: timing-only additions, world ENU e/n/u and PX4 local NED n/e/d
unchanged; authoritative snapshot validation and existing coordinate tests passed.

### Main campaign resume explicitly requested after diagnostic report

User instruction: "그럼 본 캠페인 이어서 진행해" after being told original command
hole was not reproduced/cause not confirmed,6/39 remained. This supersedes the
previous diagnostic-only stopping scope, but is NOT evidence of repair or authority
to relax QC. Explicitly announced interpretation: one reviewed P3/a03 retry only;
if it fails, stop under unchanged serial QC and repeat-failure guards. No changes
to prior attempts, seeds, XLSX, grid criteria or flight/control/QoS.

State owner now supports an explicit evidence-hashed, one-attempt user-directed
exception. Default two-failure guard unchanged; exception binds the whole reviewed
attempt chain, next attempt ID, immutable RCA/diagnostic artifacts, consumption
and user instruction.14 state tests passed incl evidence tampering, duplicate
authorization, one-use and re-block after a03 failure. Independent read review
found no material guard defect in intended locked flow. Portable/diff checks pass.
Reviewed retries also collect the optional buffered timing trace; other main runs
keep existing default. Controller launch argv retained as immutable provenance.
Coordinate contract review: status/admission metadata and timing trace only, world
ENU/PX4 local NED contracts unchanged. Main runtime verification pending.

Resume startup checks PASS:7 startup/runner-plan tests,14 state tests; existing
Axes3D warning only. Source snapshot:source_evidence/reviewed_main_resume_revision.
One-use user authorization P3/a03 recorded under runner lock, evidence hashes bound,
cause_repaired=false. Canonical configured sweep launched root session13835:
source ROS Humble + workspace/main/install + repo .venv, then
bash apps/Simulation/run_auto_attitude_sweep.sh --config
apps/Simulation/config/campaign/paper1_v1_2.env (from repo root).
Existing same-workbook campaign is resumed; valid slots preserved. Runtime pending.

Fresh configured preflight PASS, no blocking checks, original XLSX hash unchanged.
P3/a03 main attempt started after one-use authorization consumed. State readback
RUNNING/attempt_count3/authorization used=true. Existing6 valid slots retained.
Runtime session13835 remains sole owner; buffered controller trace requested for
this reviewed retry, same seed/trajectory/fixed settings and strict acceptance.

P3/a03 science completed,1323/1323 full slots covered, no unknown/duplicate/backward
reference times. Raw capture exits image0/state0/operator130. Summary_v2 actual
runtime readback:20,026 rows,zero losses/errors,producer_shutdown_verified=true.
Postprocess/QC pending session13835. The diagnostic summary's legacy static scope
string still says DEVELOPMENT; actual role is main, per run_manifest/run_status and
controller_launch.reviewed_retry=true. This label does not reclassify or promote any
attempt. Preserve original raw metadata and clarify its alias here; fix future
label after execution, not by rewriting raw evidence.

P3/a03 strict QC VALID_WITH_WARNINGS; first valid attempt counted, prior failed
a01/a02 retained. Main count7/39. Runner automatically advanced to G0-P1-R02/a01
under original workbook order/seed. This successful retry is not causal repair
evidence; original missing-slot mechanism remains unresolved. Session13835 active.

G0-P1-R02/a01 completed science1316/1316 slots and GT50Hz/no>=40ms gaps,
but state recorder exit1: RCLError failed to create timer, context invalid.
Runner skipped postprocess on invalid recorder exit and preserved failed QC/raw.
Main count remains7/39. Session13835 exited1; no automatic rerun underway.

Direct mechanism: ROS default signal handler shuts down default context during
SIGINT spin_once timeout timer creation. Recorder installed its own _stop handler
before rclpy.init(), but Humble init defaults to SignalHandlerOptions.ALL; confirmed
from installed rclpy __init__.py and signals.py. Existing expected-stop filter
recognizes wait-set invalidation only, not timer creation. Fix owner: recorder
initializes ROS with NO signal handlers, retaining its stop flag owner and shutting
ROS down only after spin exits. Does not suppress unexpected RCLError or weaken
missing-stream/data QC. Add actual ROS subprocess SIGINT check; failures and source
coords unchanged. GT ENU e/n/u xyzw, PX4 local NED n/e/d body FRD wxyz unchanged.
Also correct future timing-summary scope label; preserve P3 raw legacy label.

Recorder signal fix PASS:12 recorder/diagnostic tests passed, including real ROS
subprocess SIGINT: context remains valid at stop, missing required streams still
fail (not silently accepted). Portable/diff checks PASS. Source snapshot
source_evidence/recorder_signal_ownership_revision. Resume canonical sweep with
fresh preflight; P1-R02/a02 is an ordinary first structural retry using same seed.
No extra override granted, no main failed-attempt data rewritten.

Post-fix fresh live preflight PASS, no blocking checks. Independent recorder
review found no blocking issue; stop waits for the current callback, not an
explicit queued-callback drain (comment wording limitation only). Canonical sweep
restarted session8407; P1-R02/a02 main RUNNING,7 valid slots retained. Session13835
closed after P1-R02/a01 failed; do not poll/relaunch it. User resume authorization
remains active for serial remainder; all normal QC/repeat-failure guards apply.

### Huber explanation HTML requested during ongoing main collection

Added standalone offline Korean huber-explained.html beside the existing stereo
explanation. Controls: pixel error e, scalar sigma, dimensionless delta, synthetic
outlier position; loss/IRLS weight and1D location example. Does not call campaign
APIs or change settings. Local solver HUBER_F_SCALE1.345 and scalar-weight/whitening
implementation inspected, scalar demo distinguished from full covariance/multiaxis
fit. Coordinate contract: image-axis residual px, standardized residual dimensionless,
not ENU/NED position or optical ray. No changes to existing explanation HTML.
Checks: unique IDs/element references, JS syntax and DOM interaction harness passed;
independent hand values e4 sigma1 delta1.345 => L2loss8, Huber4.475, weight0.336;
9 centered points+10px outlier => mean1px, Huber0.149px. Zero/negative/extreme input,
reset and quadratic-region equivalence checks passed. Portable/diff checks PASS.
Browser visual QA not available (CUA returned No browser is available); temporary
localhost preview server stopped. No dependencies added. HTML is untracked inside
existing untracked v1.2 package and must accompany any desired future commit.
Main sweep session8407 was not interrupted for this explanatory artifact.

### Resume from network-interrupted G0-P3-R02

User explicitly requested continuing from the interruption. Readback9/39 valid,
P3-R02/a01 failed before CAPTURE_READY due to bag topic discovery20s timeout;
failed runtime has multicast Network is unreachable errors. No remaining owned
SITL/runner processes, current wlo1 and default route up. No claim that underlying
network cause is permanently fixed. No source/config/QC changes or special retry
exception needed: first technical failure on this slot. Same canonical configured
sweep relaunched with fresh live preflight, root execution session14525. Expected
next main attemptP3-R02/a02, same source XLSX and seed; nine valid slots preserved.
Preflight/main readback pending.

Resume readback PASS: fresh live preflight passed; G0-P3-R02/a02 main RUNNING,
CAPTURE_READY + STATE_READY and GT readiness101 unique samples/2simseconds/50Hz,
maxgap20ms. Controller entry_started; new runtime network-unreachable errors0.
Prior9 valid slots retained. Session14525 remains running the serial remainder;
normal failure stop/QC unchanged. This confirms startup recovered in this attempt,
not a permanent network repair or39-run completion. No source/config changes.

User asked to continue: existing session14525 is still active, not interrupted.
Readback11/39 valid;P3-R02/a02 andP4-R02/a01 passed strict QC. CurrentV0-R02/a01
in entry with active capture/controller; latest clock receipt age76ms and runtime
network-unreachable errors0. No duplicate runner launched, existing serial sweep
left running. V0 positioning performance not inspected/evaluated.

### V0 entry-failure investigation (read-only)

User asked why entry fails. This checks control/entry structure only; no V0 stereo
positioning error or method ranking evaluated. Failed V0 runs never approach start:
start world ENU[0,600,218.382]m versus launch near[0,415,1.03]m. Entry deadline300
host seconds expires; world-up max~1.03m and closest start distance~283m. No settle
window reached. SeparateV0-R03 failed arm before entry.

Direct initial-motion evidence G0-V0-R02: first local NED setpoint delta about
[+3.241,0.000,-3.808]m. At entry+6s reference/body pitch~-2.3deg/up1.03m; +8s
pitch~-46.8deg/up0.72m; +10s pitch~-53.7deg/up0.61m; remains grounded/tilted.
Successful P0-R01/a02 first delta[-0.516,0.000,-4.973]m, +8s up1.97m, +10s up7.14m,
near-level pitch. GT quaternion world_from_vehicle_reference_link xyzw was used
only to characterize ground tilt, not camera/triangulation accuracy.

Mechanism observed: ground-level tipping/no climb followed by entry timeout, not
small residual/stabilization failure near600m. Leading implementation hypothesis:
bounded5m 3D approach immediately requests substantial horizontal motion during
takeoff for farther V0, unlike nearly vertical P0-P4 entry. No separate vertical
liftoff stage in current entry loop. Exact PX4 internal takeoff/thrust trigger not
proven (those state/actuator fields are not retained in recorder); do not assert
geofence or218m height limit. Proposed correction for separate authorization:
vertical liftoff+height/airborne gate before transit, same V0 reference unchanged,
DEVELOPMENT validation and failure-classification review. No source/QC/raw edits,
no favorable-result reruns or V0 score inspection in this investigation.

### Active full-v1.2 goal: entry correction and evidence-based completion audit

Goal: produce results satisfying v1.2, not merely39 green record markers. Previous
goal-adjacent investigation is PROGRESS: observed V0 ground tipping and immediate
horizontal takeoff command changed the next action. Current-state snapshot34
record-QC-valid/28 completed reference trajectories; six false-valid V0 nonstarts.
Root gracefully interrupted owned runner17171/session14525 during G1-V0-R02/a01
postprocess. Controller/capture already closed, raw preserved; runner finalized
that attempt INFRA. No duplicate SITL or alternative success definition.

Slices: E1 v1.2vertical liftoff before transit (19targeted tests PASS); E2 no-science
nonstart QC classification+hash-bound downgrade API preserving old QC and raw
(38QC/status/startup tests PASS); E3 actual V0 DEVELOPMENT clearance/transit/
reference+rawQC NOT_RUN; E4 same-seed affected retries+remaining11 main slots
NOT_RUN; E5 fresh39readback/repetition audit/currenthandoff NOT_RUN; E6 existing
offline implementation compatibility/freeze/results audit IN_PROGRESS. No V0
positioning scores or method ranking opened. Existing valid calibration flights
are retained unless full QC disproves them.

Full independent gap audit found stale partial6 summary, no sequence-audit result,
no data-specific offline results. Implemented offline _status_slots omitted frozen
XLSXplanned_source and used bundledv1.1spec. Agent adapting source-awareQC first;
full derived/result scope must be checked againstv1.2 before freeze/evaluation.
Do not claim offline complete from source existence.

Entry method: hold actual launch local N/E, bounded down-only rise10m; local NED
and independent ENU rise10±0.5m, horizontal drift<=1m, speed<=1m/s stable1simsecond
with gaps<=.2s; timeout60s inside original300s entry budget. Engineering startup
procedure, not a changed V0 trial path; runtime truth remains entry validation
only. Protocol-version CLI1.2 activates it; legacy1.1unchanged. Source gate owns
clearance, flightcontroller sequences, existing start-gate owns trialreadiness.
Rootrunner passes frozen source protocolversion and hashes new entry_liftoff.py.
Portable-path/diff checks PASS; freshfull campaign test suite RUNNING.

E2 runtime metadata correction PASS: six original false-valid V0 attempts fully
fresh-QC checked then downgraded under lock. Before-state backups and report in
source_evidence/v12_entry_reclassification_001; oldQC/runner/checksum snapshots
inside each attempt; all raw retained. count34->28, all28 now completed science
trajectories. API guard additionally rejects empty/incomplete original checksum
coverage.18status tests PASS including tampered source, in-science protection,
empty hashlist and same-seed nextattempt. No automatic rerun of valid science.

Full regression initial unsourced command230passed/1failed: realROS signal test
could find rclpy but not workspace px4_msgs. This is an environment prerequisite
miss, not a demonstrated source regression. Re-running fullsuite with both
/opt/ros/humble and workspace/main/install sourced (same runtime environment).
Offline source-aware QC integration9tests PASS, calibration_contract/reprocessing/
split_policy JSON deep-equal to bundled protocol; still no offline execution or
V0 positioning scores. MC representative-only scope remains to audit.

Configured full regression PASS232tests/123.78s; existing Axes3D warning only.
Agent offline matrix10targeted tests alsoPASS (overlap not added to aggregate).
Main comparison MC now covers all declared calibration runs/yaw cases/both sample
scopes per trial; no subset substituted for the requested main matrix. At100trials
GPS additions imply42,400batches/254,400mainfits; no compute or V0score opened yet.
E3 DEVELOPMENT G0-V0-R01 started using same frozenXLSX/seed, separate development
namespace, new liftoff source snapshot source_evidence/vertical_liftoff_revision_001.
No runtime policy/source edits during this trial; inspect events+control readback,
not final-positioning scores.

E3 first DEVELOPMENT flight partially passed: verified verticalrise local9.814m/
world9.730m, subsequent transit/start-gate passed and V0science completed.
However strict2378-slot cadence covered2377, missing700 [123.152,123.172)s.
QC INFRASTRUCTURE_FAILED solely science_setpoint_cadence_below_v12_requirement.
No mainpromotion, no QC weakening. Originaldevrun/failedQC retained.

Reproduced causal boundary: callback123.132s held synchronous reference-update
operation54.839ms (lock acquisition7.192us; total54.964ms). Independent clock
received all five missing-slot source ticks while observer serviced none; next
accepted snapshot123.176s skipped toslot701. Not GT rejection/initial lock wait.
Exact54ms suboperation unobserved. Entry defect improved; command scheduling
remains unverified and blocks mainrestart.

Next fix: prepare/validate immutable reference before arming; cached timestamp
binary search uses same arithmetic.501archivedV0queries exactly equal legacy.
Microbenchmark median threadCPU550.651us->21.468us, not runtimeproof. Add optional
wall/threadCPU profiles of prepare/send/postlog and GC callbacks, no GCpolicy,
QoS, RTF, historical send, or acceptance changes.30targeted tests PASS.
SecondDEV trial must pass actual fullcadence/QC before mainrestart.

Actual first-science raw left/right PNGs extractedlosslessly with pixelhash equality
at vertical_liftoff_revision_001/visual_qa; both show complete actualdrone mesh
inside image with broad margins. This is two-frame visualproof only, not whole
pathvisibility proof. Offline uniform-scene mesh pixel audit implementation tasked
separately; it must report unsupported scenes instead of fabricated detections.
Updated partial28summary saved, V0positioningrevealed=false.

Prepared-sampler/timing integration PASS71tests/30.99s. Source-aware analysis+
selected-XLSX main-run command+mesh synthetic checks PASS21tests/4.82s. Selected
run --frozen-workbook now supports main attempts as required (same frozen-source
validation and normal validity/retry guards); --development remains optional for
non-counted diagnosis. No additional manifest operator input.

Originalv1.2JSON/txt frozen into frozen_protocol with content hashes; offline loader
requires actualv1.2schema/version instead of silently selectingbundledv1.1. Actual
protocol and frozenXLSX/compiledplan/schema mapping hashes bound in sourceidentity
and freeze. Legacy provenance clearly distinguished. Sensor generator internally
shares scientifically identical calibration_contract/reprocessing/split_policy;
future differing contracts require adaptation, not an implicit fallback.

Offline mesh audit5synthetic checks PASS; actualfirstDEV visibility scan active
session31299. No flight concurrent with scan. Main valid28; no mainrestart until
secondDEV timing check passes. All existing recording/schema/seedfailure history
retained; no measurement V0 score or algorithm ranking opened.

Actual firstDEV rawpixel visibility PASS_UNDER_ASSUMPTIONS:952scienceframes per
camera, allVISIBLE, minimumedge201px both; originalpixelSHAverified. Threshold
32px is engineeringguard, sceneidentity/full3Dmesh not inferred from mask alone.
Combined with rawvisualquadcopter inspection/uniformscene this supplies actual
rendered-silhouette margin evidence; preserve assumptions in finalreport.

E3 secondDEV G0-V0-R01/a02 launched sole session94533 after all flight-dependent
checks finished and actualmesh scan31299closed. Sourcearchive002 includes prepared
sampler+stage wall/threadCPU/GCobservations. Goalcheckpointupdatedwithallphases/
remaining work. Main remains28completedreference trajectories,7failedV0 slots,
4planned. No mainrestart or frozenV0accuracy evaluation until current strictQC
and fullsource collection prerequisites pass. Goal staysactive.

E3 secondDEV PASS: full QC VALID_WITH_WARNINGS, no infra/protocolerrors, both
campaign inclusion flagsfalse. Science2378/2378fullslots,unique source50Hz, max
receive-to-send2.681ms. Trace39714rows/loss0/errors0/shutdownverified. Stage
maxwall prepare1.781ms/send2.347ms/postlog.926ms;57GC intervals duringSCI(g0/g1),
max2.037ms,no generation2. This is actual qualifying trial, not permanent proof
of absence of all OS delays. Detailed timing_analysis.json retained.

Mainv1.2 now retains same optional timing/GC evidence as validatedDEV, so future
rare failures remain diagnosable. No GCpolicy/QoS/reference/acceptance change.
Eight startup/selectedrun tests PASS after diagnostic composition; portable/diff
checks PASS. Canonical configured main sweep launched sole session93774 with
fresh preflight and sourcearchive corrected_entry_main_resume_001. Expected
first correctedmain G0-V0-R01/a02;28previousvalidscience retained.

E4 actual correctedmain progress: freshpreflightPASS, G0-V0-R01/a02 completed
science and strict2378/2378cadence; captureimage0/state0/operator130; finalQC
VALID_WITH_WARNINGS. Main29/39, nextG0-V0-R02/a02 running session93774.

Independent scientificsolver audit found actualcontract mismatch before any
V0scoring: globalHuber loss robustifiedIMU prior contrary to rTinvSigma_r. Fixed
blockloss: imageHuber; IMUquadratic and declaredGaussianRTKquadratic forB, consistent
innerlatent/outerobjective. FixedXYZ/ApropagatedRTKcov/Cinputallowlists unchanged.
Added data-only versusIMU-included rank/condition/singularvalues, explicitly
profiledattitude scope forB/C, notfull latentstate rank. JSONsafe nullcondition
forrankdef verified by source/test; 22synthetic solver/orchestration testsPASS.
README/methodology/freezepolicyupdated. No actualdatafit, V0performance, newyawprior
or data-dependenthyperparam tuning. Rootreadreview confirms requiredquadratic
prior anddiagnosticfields nowrepresented. Finalalgorithmfreeze still pending
whole-source39QC/developmentvalidation; sourceexistence not resultcompletion.

### Source-step repair and actual recorder recovery (2026-10-18)

Main session93774 terminated exit1: G0-V0-R03/a02 completed its reference but
strict cadence missed slot1066. Main valid30/39; no live main flight. Preserved
raw source clock shows all ticks while controller delivery skipped intermediate
ticks after host stall. Prepared sampling alone is insufficient. Selected native
Gazebo pause/multi_step barrier, unchanged4ms physics/reference/gains; no historical
setpoint replay and no relaxed QC. New simulation_stepper/verify_simulation_steps
source plus controller observer barrier requires actual ground probe then fullDEV.
Preflight now invokes probe after STATE_READY using isolated pure-Python protobuf
backend, hashes report, and fails on failed probe. Actual service semantics NOT_RUN
until live report. Unit/startup regression session35454 running; live no-flight
preflight session24439 running, not main restart.

Actual forced-recorder recovery session14070 terminal exit0, report
recordings/stereo_v1_1/paper1_v1_2_001/개발시험/forced_capture_recovery_001/capture_recovery_report.json:
PASS_DEVELOPMENT_ONLY. Synthetic ROS domain89 source, owned bag child killed, failure
detected, overwrite refused, new attempt recorded successfully, old hashes unchanged.
No flight or main source added, no V0 positioning scores opened.

Ground preflight session24439 terminal exit1: a010 probe FAIL before arming,
`simulation step control: service request not acknowledged`; no actual step
semantics proven. Report/log preserved under preflight/short_capture_G0-P0-R01_a010.
Owned stack exited130 with cleanup. Space-for-full-campaign NOT_MEASURED because
probe failed, not disk exhaustion (free~205GB). Investigate service discovery/IP
before newflight. Regression12 tests plus adapter2 PASS; portability/diff PASS.

### Service discovery repair (2026-10-18)

RCA: fresh persistent Node first200ms request failed; same Node second request
succeeded~1.9ms. Minimal owned Gazebo Sim8 diagnostic world (no vehicle/rendering)
reproduced this using campaign Python/backend. service_info discovery took~2s,
then first pause ACK~103ms and subsequent commands~1ms. No service/world-name
mismatch established. Corrective change: resolve control service before controller
arming/probe, bounded discovery; keep command200ms and never retry ambiguous
multi_step. Real service-only repaired probe PASS_SERVICE_ONLY (actual physics
progress deliberately unclaimed), evidence:
recordings/stereo_v1_1/paper1_v1_2_001/개발시험/gazebo_step_service_001/service_discovery_repair.json.
Owned minimal server session82411 closed exit0 after SIGINT. Actual full-world
preflight session84131 now running; previous a010 preserved.

Full-world preflight84131 terminal exit0 PREFLIGHT_PASS. a011 actual clock
20.848s pause ->20.868 ->20.888 ->20.892 then resumed20.896s; unchanged epoch0
and reset counters4/2, observer shutdown verified. Report hash bound in preflight
report; environment snapshot source_evidence/source_step_revision_001. All15
stepper/startup/adapter tests PASS (30.43s). Full DEVELOPMENT G0-V0-R01/a03
session13527 launched with exclusive runtime sourcezip/hash inventory; no mainretry.

Reference generator freshly ran against originalv1.2spec: REFERENCE_GEOMETRY_PASS.
Initial exact JSON equality assertion failed; inspection quantified only numerical
leaf rounding max5.684e-14 across six packaged paths (matching shapes/strings).
Preserved explicit exact-equality failure in regeneration_audit.json, no packaged
files overwritten. Thirty current accepted runtime references compared to fresh
author-generator samples: max1.137e-13, duration agreement within1e-10 engineering
comparison tolerance. source_evidence/reference_regeneration_v12_001 reports;
PARTIAL30 only, not final39 and not actual mesh/dynamics proof.

DEV13527 reference completed and controller exit0; captures image0/state0,
operator130. Strict cadence preliminary recomputation2378/2378fullslots, no
missing/unknown/out-of-range/duplicate/backwards references; unique50Hz, max
sourcegap20ms. Native barriers2379, all exact observed target, endpoint overshoot
889101ns (<4ms). Source-pixel science frames952/side. Buffered diagnostic73242
records, loss0/error0/shutdown verified. Worst receipt-to-send90.977ms retained:
source pacing prevents skipped source-time slots; it does not promise low host
latency or PX4 actuation timing. Full YOLO/alignment/rawhash QC still running.
Source evidence source_step_revision_001/development_step_execution.json.
README updated native service discovery, pause/step barrier, protobuf process
backend and verified vertical startup behavior; no new CLI/input file required.

DEV13527 terminal exit0, fullQC VALID_WITH_WARNINGS, no infra/protocolerrors;
countedfalse. Existingwarnings some_images_lack_valid_gt (allattempt endpoints)
and stochastic_repetition_independence_unverified retained. Repaired main retry
review bound sources/preflight/DEVQC/checksums, R03/a01 armrejection still not
internally explained, a02 cadencecause addressed. Existing one-use state API
authorizes R03/a03 under userfullgoal/resume instruction and spec repair/resume
rule; cause_repairedfalse retained as no blanket/permanent certification, strict
QC unchanged. Oldattempts/seeds preserved. Sole main resume session30677 launched
using configured runtime env, frozen workbook, existing passing preflight; starts
G0-V0-R03/a03 then remaining8. This is no longer DEV. No V0scores opened.

Clock-domain caution investigated: DDS-exported PX4 timestamp delta65.17s while
science source delta47.56s is not onboard clock evidence. Installed primarysource
uxrce_dds_client/dds_topics.h.em passes session time_offset to uCDR, whose
Tools/msg/templates/ucdr/msg.h.em adds agentoffset to timestamp; codebuild.ninja
defines ENABLE_LOCKSTEP_SCHEDULER, GZBridge uses actual msg.sim time to set
monotonic PX4 clock, drv_hrt.cpp routes that to lockstep scheduler. Do not infer
physical clock drift from DDS-adjusted timestamp, or mix it with ROS source ns.
Existing recordings intentionally keep these separate clock domains.

### Actual model geometry readback (2026-10-18)

Found completion gap: vehicle dimensions were still unmeasured/null. Read-only
native installed gz-common5.8 MeshManager measurement of actual COLLADA vertices,
SDF scale/link/visual poses and primitive boxes/cylinders now recorded under
source_evidence/vehicle_geometry_v12_001 with C++/Python sources, vertex dumps,
report and hashes. Rest visual enclosingbounds2.4856x2.4856x1.3348138481m;
full z-axis rotor-sweep enclosingbounds2.4874389296x2.4874389296x1.3348138481m.
Conservative cylinder enclosing boxes disclosed; not a physical survey or dynamic
mesh-visibility proof. Native vertex extrema match MeshMin/Max; independent
rotorcenter/propbox calculation agrees1e-12m. Source and actual Simulationinstall
SDF/DAE assets hashes equal; initial wrong message-overlay lookup corrected and
disclosed. Semantic reference zero relative base_link verified; navsat defaults
recorded, physical antenna lever arm remains null. README current summary updated.
No vehicle/runtime/config edits. Main30677 alive; R03/a03 reference completed,
controller0/capture0/state0/operator130, offline detection/QC still running.

First repaired MAIN G0-V0-R03/a03 fullQC VALID_WITH_WARNINGS; strictcadence
2378/2378fullslots/unique50Hz; no infra/protocolerrors. Main31/39. Priorfailed
attempts preserved and reviewedretry consumed exactlyonce. Sole session30677
continues G0-V0-R04/a02. No V0positioning scores read.

### Offline sensitivity coverage correction before freeze (2026-10-18)

Final spec/text audit found another scope weakness: generic reprocessing.mc_repeats
100 has no single-trial sensitivity exemption, while existing implementation
uses only G0 repetition1/trial0 for each25 OFATlevels. Prior ledger explicitly
disclosed that narrow implementation; this supersedes it. Apply full100 MC to
every existing G0 P0/selected-path repetition (no extra physicalruns, no full
factorial). This preserves shared rig/repetition/trial sensor draws and same-V0
repetition pairing, and remains before freeze/V0scores. GateS1: writer coverage
allchosenruns x100 x25; gateS2 independent auditor rejects missingrep/trial; gateS3
mocked orchestration exercises real persisted cases and repeat-specific pairing.
Main30677 running unchanged; no scientific fits/evaluation executed yet.

Main30677 terminal exit1: G0-V0-R04/a02 before science_start, originalraw
retained/QC INFRASTRUCTURE_FAILED; main31/39. Diagnosis preserved in
source_evidence/paused_start_R04_a02/cause_review.json. Actualclockquiet
112.100s for2.011686hostseconds/1419messages disproves wording that clock
didnotquiesce. Code advances quiettimer only after coherent cache.snapshot;
independent GT/localreceive skew107.516ms exceeds unchanged100ms gate, no new
local whilepaused. Exactcontrollercachepredicate notlogged; inference disclosed.
Cadenceowner assigned bounded repair (prearm actualroundtrip warmup; rawclock
quiet versus snapshot diagnostics; max2 pauseattempts, each recoveryresumes and
clears/recollects original5sstablegate within originaldeadlines). No mainrestart
or relaxedfreshness/skew/readiness. Required actualprobe/DEV before retry.

OfflineS1/S2 implementation complete: every G0 P0/selected-path repetition gets
configuredMC count,25OFATlevels; independent auditor expects25000cases when
selectedpath!=P0 (12500 if P0). Main42400batches unchanged. Initial20lighttests
passed before expanded evaluator handoff; added evaluator/CSVtest initially
failed for syntheticfixture lacking attempts metadata, corrected fixture. CSV now
keeps trial and physicalrepetition separately; summary separates path and counts
physicalreps/MC/evaluationcases (no falsepath_count). Finalrerun pending.

Offline sensitivityS1/S2/S3 PASS20tests/5.38s after fixture correction. Synthetic
executor persisted8cases and48method/variant evaluatorCSVrows with2paths x2
physicalreps x2MC, same-repetition V0 pairing; independent auditor acceptsfull
25000case set and rejects missingcase/wrongV0. Metadata keeps trial/repetition
and path groups distinct. Sourcezip/hashes/testscope under
source_evidence/offline_sensitivity_revision_001; actual100MC stillNOT_RUN, no
realV0data evaluated, no algorithmfreeze claimed. Portablepath/diff checksPASS.
Independent actual-recorder replay also PASS (paused_start_R04_a02): freshclock
plus ONLY pair_receive_skew rejection atpause+150ms, originalgatesunchanged;
not actualcontroller receipt reconstruction. Runtimefix source under review;
root caught warmupconstantclock fallthrough, owner corrected/tested, then asked
for fresh pausedclock receipt progression (single cachedmessage insufficient).

Combined campaign regression PASS286tests/115.72s across39files, both ROS and
workspace overlays sourced, root .venv; logs/XML/sourcezip+hashes in
source_evidence/paused_start_revision_001. Includes actual ROS signalchild test,
prior/conditioning/finitecorrelation diagnostics, MC orchestration and new pause
admission/freshreceipt guards. ExistingAxes3Dwarning only. Review caught/fixed
warmup fallingthrough to quiescence and single cachedclock acceptance; negative
tests now require real advancingclock for warmup and refreshed callbackreceipts
through100ms quietwindow. No gate thresholds/QoS/physics/path/gains changed.
Actualfullworld preflight newly launched; no mainflightrestart.

Completed explicit textmetric gap: solver now writes attitude_correlation with
6x6 matrix/order/undefinedaxes/rank/nonfinite diagnostic metadata, no objective
change; fixedvariance axes undefinednull, rankdeficientcovariance allnull with
reason. Summary separates offlinecasecount/rawcalibrationrun/evaluationrun/MC
identities, explicitly reports attemptednonconvergence vs missinginputs.
Earlier explanatoryHTML still described outdated globalHuber on IMU/RTK:
corrected B formula and both HTMLnotes to image-onlyHuber + quadraticGaussian
IMU/BRTK consistent with code (text/formula only; no interactiveJS change).
No actualcalibrationfit/V0evaluation on this dataset; freeze stillpending.

Actual preflight31758 terminal exit0 PREFLIGHT_PASS, a012 native observed
servicewarmup/clockadvance + actualfreshreceipt quiet +20/20/4ms/resume and
unchangedgeneration, observerjoined. Environment snapshot copied under
paused_start_revision_001. Development18825 now runs G0-V0-R04/a01 using
EXACT failedmain condition/seed from frozenworkbook, preserved separateDEV.
Runtime sourcezip/hash bound; no mainretry yet.

Pre-freeze evaluator review found real mixed-validity bug: estimates filtered
by validmask but trutharray notfiltered. One invalidpoint could produce
broadcastshape error caught as totalreconstructionfailure. Applied samemask
to truth, syntheticvalid+NaN case nowretains1valid/1failure, conditionalRMSE1m
andavailability.5 as handexpected. No actualV0data touched. This ONEline
analyzer change is AFTER286fullregression; targeted22testsPASS6.46s. New source
snapshot offline_metrics_revision_001 disclosesversiondifference; runtime
FROZEN sources unchanged so actualflight qualification remainssamecode.

### Full-matrix execution memory boundary (pre-freeze,2026-10-18)

Source feasibility review (no actualfits) identified growth with42400mainbatches
and up to25000sensitivitycases: per(run,trial) inputcache3100copies retained;
V0 cache up to1600duplicate GTseries; allrich calibration/evaluationpayloads and
flattened XLSXrows additionally retained. Cannot infer30GiRAM/25Giavailable
is sufficient from20logicalcores/190Gi diskfree. Inodes checked112,471,532free;
no inodeexhaustion observed. No OOM or timingbenchmark claimed.

Boundary decision: keep existing immutable per-case JSON/checksum files as
resultauthority; coordinator alone owns thinindexes/freeze/latestpointer; per
run/trial inputs expire, evaluator uses bounded attempt-keyed staticinputs,
reporters repeatedly stream validated disk cases and onlineaggregates. Preserve
all matrix/values/methodfiles/seeds/selection/freeze-before-V0/resumehash behavior;
no newoperatorfile/framework or parallelworkers yet. Existing owners kept; no
resultownership transfer to exporter. Assigned offlineowner analyzer/workbook
streaming and lighttests, no runtimeedits/actualdatasetfits/V0scores. Root owns
actualDEV18825 and later G0-only quiet-window time/RSS/disk benchmark plus
independent finalschema coverage. Actualdata fit stillNOT_RUN.

DEV18825 terminal exit0 G0-V0-R04/a01 fullQC VALID_WITH_WARNINGS, no infra or
protocolerrors,2378/2378fullslots. Actualpaused snapshotacceptancefirstattempt:
skew15.841685ms underUNCHANGED100ms, freshages124/140ms under500ms. Noactual
recoverytrigger; boundedrecovery remains unittested (not claimedflighttriggered).
Reviewed R04/a03 one-use authorization binds failedrawRCA/replay, updatedsource
zip/hashes,286runtime regression, actualpreflight and DEVQC/checksums. Prior
failedattempts/seed preserved, cause_repairedfalse remains noguaranteecertification.
Sole MAIN session24439 now launched R04/a03 thenremaining7. This handle number
was reused after oldpreflight24439 terminated, checkpoint distinguisheshistory.
Main31/39; noV0positioning scores. Runtimehashes ALL MATCH full286regression;
later analyzer streaming edits do not touch runtime and need ownverification.

### Arming acknowledgement/control lifecycle (2026-10-18)

MAIN24439 latestgeneration terminal exit1, R04/a03 INFRASTRUCTURE_FAILED
beforeentry/science.31valid remains. Controllerfirst2ACKtimeouts5s, then2
TEMP_REJECTED1. Independent source defect confirmed: legacy arm() stops
setpoints during5sACK waits and2sretry sleeps; GcsHeartbeat is not offboard
setpoints. Installed PX4 offboardCheck requires freshnessCOM_OF_LOSS_T
(defaultsource1s). Initial ACK loss/denial exactcause notclaimed fromapp log.
Captureowner investigating associatedtargetULog, control/params/messagesonly.

New v1.2 offboard_arming owner keeps ORIGINAL held NED/yaw at50Hz across all
ACKwait/retrydelays;4attempts/5swaits/2sdelays preserved. Source+commandfilter,
STATUSTEXT/ACK/HBstatus and sendgap evidence; only actualtargetARMED+OFFBOARD
heartbeat succeeds. No forcearming/no parameters/nohealthgate bypass.
Legacyv1.1 unchanged, cancellation/safeLAND retained.20focusedtestsPASS.
Root wires FROZEN/provenance and perattempt PX4console target/camera logs;
futurehealthreason notdiscarded to/dev/null. No newCLI/operatorinput.
Combined40file regression77338 running on archivedsources in
source_evidence/offboard_arming_revision_001 (also includes completedbounded
streaming implementation), afterwhich actualpreflight/DEVrequired. No repeat
maintry before evidence; nextR04/a04 stillneedsreviewedauthorization.

Offlineowner stable boundedserialimplementation: perrun/trial inputexpiry,
thin caseindex/path/hash, oneattempt staticGTcache, repeatedchecked diskcase
iterator, streamedCSV/onlineexactmeans, two-pass XLSXheader/rows with
list/factory parity and changedsource detection. calibration_run_index_v2
keeps identities +case_directory/calibration/evaluationhash. Fullmatrix/freeze
order unchanged.32targettests pluscorrectedfixturehandoff1PASS; root40file
regression nowincludes them. Thinindexes/finalchecksuminventory stillscale
with filecount, G0-only RSS/time/diskbenchmark REQUIRED beforefullactualfit.
Noactualdatasetfits/V0scores opened.

Combined current-source regression PASS300tests/118.65s,40campaignfiles,
bothROS/overlay sourced. Root rechecked EVERYarchivedsource hash aftertests;
source stable. ExistingAxes3Dwarning only. Includes streaming fullsynthetic
freeze/diskiterate/CSV/XLSX/pointerhandoff, exactmeans and list/factorybyteparity,
inputcacheexpiry/oneGTattemptcache, arming20cases, currentmixedvaliditymask and
correlation metadata. Actualflight qualification NOT_RUN on this armingrevision.

ULogforensic evidence strongly associates target18_42_56.ulg with R04/a03
using2150exactfour-component WXYZmatches, compatiblesourceclock/bootUTC/reset5,2
andmtime; no embeddedrunID, associationbasis disclosed. Rootverified preserved
SHA cc5919671d8b9d4eb999707cde9c660c942359e07221503fd0051fad23ef7470.
source_evidence/offboard_arm_R04_a03/forensic_report.json +ULog retained.
ActualCOM_OF_LOSS_T1.0s (unchanged IN THISLOG). TargetARMED+commandACK0 at
25.568s; lastoffboardupdate25.540s, signallost26.552/failsafe26.564, auto
preflightdisarm36.580. Thus appACKtimeout did NOT mean targetneverarmed;
waitingwithoutsetpoints explains observedoffboardloss. InitialmissingACK
transportcause and exactlaterTEMP_REJECTEDreason remainunproven. No forcedarm
or changedhealthparameters; nextprobe/DEV mustconfirm actualsustainedstream
and targetARMED+OFFBOARD heartbeat. No3Dpositionmetric/fit/rankings read.

DEV28887 R04/a02 failedONLY modeformatter gate: actualARMEDtrue,base145,
custom393216(main6), but installedpymavlink interpret_px4_mode returnedUNKNOWN.
Root reproduced with actualHEARTBEAT wirefields; officialPX4custommode.h main
bits16..23 OFFBOARD6. Corrected sourcegate requiresPX4+CUSTOM_MODE_ENABLED+6,
keepsactualARMED/sourceidentity. Real encoded-and-parsed goldenpacket and
wrongmain/missingflag/wrongautopilot negatives pass; rootfocused45tests/27s.
No newphysicalprobe needed fordecoder-only change; reuseda013 unchangednative
world/stepper/capture proof, sourceversiondelta explicit. DEV81915 R04/a03
thenFULLPASS VALID_WITH_WARNINGS, no infra/protocolerrors, actualmain6/ARMED
ACK0firstattempt,15heldsends/maxgap42.832439ms,verticalclearance and2378/2378
sciencefullslots. Source/runtime/evidence decoderrevision bound. Reviewedone-use
R04/a04 binds failedULog/300priorregression/newdecoder/actualDEVQC+checksums,
allfailedraw/seed preserved, noforce/parameter changes. SoleMAIN16445 resumed
R04/a04 thenremaining7;31valid/39, noV0positioningscores.

Found/corrected extra matched200timecoveragegate beforefreeze: oldselector
returned0for200unique pairs withtemporalhole, conflicting explicit<200unmet
rule and independentauditor. New exactinteger monotoneassignment minimizes
absoluteerror tosameequal-time-bincenters and alwaysreturns200forvalidN>=200;
actualgaps/targetdeviations separate, malformedduplicates/reusedframes/epochs/
timestamps inputerror. Cachedmask shared A/B/C/variants/yaw/MC whereimmutable.
Root31testsPASS14.11s inclsmallbruteforceoracle,pure200gapcase,199case and
realcal/evalinputhelpers; no actualsourcefit/V0data read. Sources/evidence
matched_200_revision_001, sourcehash2a6bb0...

Recovered BOTH executedgeneratedWorld XMLbytes matching recordedcameraTruth
SHA forALL31currentvalidruns (G0/G1), preservedfromownedtemporarySDFfiles
under generated_worlds_verified_001 with hash→runmapping. XMLconfirms both
cam1280x720,near0.1/far1000 andtargetURI model://x500_2m. Referencedexternal
meshasset history remainsseparate; no strongerclaimthanmatchedworldbytes.
Added mesh_enclosure_audit +4tests: actualsourceCI/staticpose/GTlinear<=40ms
andall-attitude source-modelenclosure QA; notactualvisibility/accuracyproof,
actual39scan pending. No runtime/analysis changes from thisaudit.

MAIN16445 live poll: G0-V0-R04/a04 controller exit0, capture image0/state0, operator130; final QC VALID_WITH_WARNINGS. run_status confirms32/39 valid. Same owned runner automatically started G0-V0-R05/a02; no duplicate launched. No offline V0 positioning metrics read.

Partial handoff inventory at32valid: source_evidence/partial_required_artifact_inventory_032.json binds current spec/status snapshot hashes; all15 required file names exist for first valid attempt of32slots, first nonblank row on both observation streams contains all16 required fields. This is existence/first-row evidence ONLY, not whole-record/hash/quality completion. Live MAIN16445 R05/a02 passed vertical clearance and remains in normal entry. No actual fits or V0 positioning metrics read.

MAIN16445 R05/a02 science completed and postscience hold observed159.404->164.432 source s (5.028s); controller0,image0,state0,operator130; final QC pending. 32completed slots first-valid attempt equals latest attempt; repetition auditor latest-attempt selection is equivalent for current valid sources, must recheck at39. Disk189GiB free; no capacity claim for full offline matrix.

MAIN16445 R05/a02 QC VALID_WITH_WARNINGS; status33/39 valid; same runner started G1-V0-R01/a02. Actual sourcecamera world SHA bindings rechecked for all33 completed slots against archived SDF bytes: generated_worlds_verified_001/completed_033_world_bindings.json. Filename initially032 during concurrent completion, immediately renamed033 to match recorded count; no raw changed. Includes new R04/a04,R05/a02; model asset historical identity remains separate. No positioning scores opened.

R05/a02 strictQC detail reviewed: each camera952scienceframes/47.568s, maxgap52ms; GT50Hz/no40ms gaps, all1904 scienceimages EXACT/INTERPOLATED and rawbag bracket list empty; cadence2378/2378 slots/zero missing, true50Hz, RTF0.73747. Warnings some_images_lack_valid_gt applies328 EXTRAPOLATION outside SCI (science EXTRAPOLATION0); stochastic repetition independence remains explicitly unverified. No infra/protocol errors. G1-V0-R01/a02 actual vertical clearance completed; MAIN16445 live.

Live MAIN16445 G1-V0-R01/a02 passed vertical/start stabilization and completed science; continuous physics resumed, controller0,image0,state0/operator130. Postprocess/QC pending;33/39 maintained, not counted early. No duplicate or altered gates.

MAIN16445 G1-V0-R01/a02 QC VALID_WITH_WARNINGS, no infra/protocolerrors; actual status34/39 valid. Same runner started G1-V0-R02/a02, remaining5 including active. Full remaining scope unchanged: raw39 freshQC/hashes/reference/repetition/pixel+enclosure/handoff, current-source regression, G0-only measured analysis benchmark, full100MC+OFAT and freeze-before-V0/finalresult audit. No V0positioning metrics read.

MAIN16445 G1-V0-R02/a02 passed actualvertical/startgate and completedscience, controller0/image0/state0/operator130; postprocess/QC pending.34valid remains untilQC. Confirmed current campaign_run.FROZEN_FILES includes entry_liftoff.py,simulation_stepper.py,offboard_arming.py; per-attempt codehash capture retained. No runtime source mutation or fits/evaluation duringflight.

MAIN16445 G1-V0-R02/a02 finalQC VALID_WITH_WARNINGS; no infrastructure/protocolerrors; status35/39valid, cadence2378/2378fullslots. Same runner automatically started G1-P0-R02/a01, remaining4 includingactive. Source first-valid rule preserved; no repeatedperformancefailed run/noV0position scores opened.

MAIN16445 G1-P0-R02/a01 completedscience, controller0/image0/state0/operator130; postprocess/QC pending,35/39 maintained. Reviewed finalQC CLI supports --planned-source-json and --report-path outside immutable raw; visibility/enclosure CLIs reject attempt-contained output/overwrites. Actual39 fresh checks remain NOT_RUN until collection quiet. No analysis scores opened.

MAIN16445 G1-P0-R02/a01 QC VALID_WITH_WARNINGS, zero infra/protocol errors; status36/39valid. Same runner started lastV0 slot G1-V0-R03/a01; then G1-P0-R03 and G1-P4-R03 remain. Checkpoint persisted. Allraw39/finaloffline100MC/finalaudits remain required; not complete.

MAIN16445 G1-V0-R03/a01 lastV0 science complete; controller0/image0/state0/operator130, postprocess/QC pending;36valid maintained. Existing end-of-campaign _audit_valid rechecks files/hashes using status-slot parameters; final independent saved39QC should bind original frozen workbook_plan.planned_runs instead, and persist perrun reports outside immutable attempts. Last-valid/first-valid equivalence to recheck at39. V0 accuracy remains unopened.

MAIN16445 G1-V0-R03/a01 QC VALID_WITH_WARNINGS, zeroinfra/protocolerrors.37/39valid, all8V0 rawslots valid; no V0positioning scores opened. Same runner started G1-P0-R03/a01, then lastG1-P4-R03/a01. Mainattempt census beforelastQC:15historicalINFRA attempts preserved separately fromvalidslots/development; finalcensus mustrefresh. Goal/fullanalysis/handoff scope unchanged.

MAIN16445 G1-P0-R03/a01 VALID_WITH_WARNINGS, no infra/protocolerrors; status38/39valid; automatically started LAST G1-P4-R03/a01. Current sourcedcampaign regression selection41files (new enclosure test plus corrections afterfull300); actualcurrent fullrun NOT_RUN untilquiet. Fullraw39/resultscope unchanged.

RAW39 COLLECTION COMPLETE: sole MAIN16445 terminalexit0; WORKBOOK_CAMPAIGN_COMPLETE, frozenworkbookSHA3973763f...,planned39,valid_readback39,statuscounts VALID_WITH_WARNINGS39. LastG1-P4-R03/a01 QC passed; runner actualfreshall39 file/hash readback passed. This completes collection-stage gate only; fullgoal remains active pending independentfrozenplan reports/current-source regression/reference/repetition/visibility/rawhandoff and full100MCcalibration+sensitivity/freeze/V0evaluation/finalaudit.

Independent frozenplan39 QC session61895 terminal0 PASS39; originalfrozen workbook SHA/compiledplan integrity checked and each FIRSTvalid attempt equalslatest; each report saved final_raw_qc_001, rawunchanged. Current41file regression98228 live. Firstactualenclosure23983 terminal0 conditionalPASS G0-P0-R01,526frames/side,minmargin203.737px; report final_visibility_001. Full39 two-worker CLI pixel+enclosure batch90200 nowlive. Initialbatch launch exit1 ModuleNotFoundError campaigns frompackagecwd beforemanifest/worker mutation; repaired sys.path Simulationroot and reran, no raw effects. Assumption-limited geometry/pixel QA, no accuracymetrics.

Current campaign regression98228 terminal0:41files,312PASS132.43s (runner135.59s), existingMatplotlibAxes3D warning1; no skipped/failed. Source/tests ZIP+hashes,exactargv/log/JUnit/result current_campaign_regression_001. Covers allchanges sincefull300 includingrealwiremode decoder/matched200/enclosure. Repetition audit40925 launched onlyafter39/noSITL; pixel/enclosure90200 stilllive. Actualfull100MC fits remain NOT_RUN.

Repetition40925 terminal0:9groups,0identical fullsourcepixel+GT sequences; NOT proof stochastic independence. All39 reference fields/samples matched original generatedauthor references maxabs1.136868377e-13 underlabelled1e-10 numericalcomparison tolerance (no flightgatechange); main_39_reference_comparison.json. Bothcamera generatedworld hashes bind archivedSDF for39; completed_039_world_bindings.json. Pixel/enclosure90200 first2pairsreportsPASS; remainingongoing. G0-only benchmark34100 started singleprocess/BLAS1, G0-P0-R01trial0 all8yaw x2scopes; separatebenchmarkoutput, noV0positionerrors, cameraobservationgenerator camera-pose checks across39 explicit. Actualfull100MC not yetrun; decisions must use measuredcost/RSS/disk.

G0 benchmark34100 terminal0:16batches (G0-P0-R01trial0 Y0..Y6+GPS_YAW90 x all_data/matched200),136.53879s,peakSELF RSS113876KiB,418files/logical35898334bytes; actualfit settings/matrix unchanged; noV0positionerrors. Benchmarkmanifest bindsalgorithm/protocol. RTK JSONL dominates:20221296bytes32named drone_rtk_observations plus10005488bytes16yawideal copies; free199351365632bytes observed. Need measured full-run storage forecast inclsensitivities/evaluation/export/reserve, not assume190GiB ample. Raw capture_summary_39_preanalysis.json produced39runs/9groups, V0metricsfalse/offlineNOT_RUN.

Independent read-only offline_audit review: safe candidate main workunit(run,trial), camerageneration singlecoordinator beforetasksubmission, all yaw/scope fits withinowner; sensitivity unit(scenario,run,trial), pathselection completefirst. CoordinatorONLY index/pathselection/freeze/latestpointer; deterministic existingtrial-slot-yaw-scope order, boundedspawnqueue/BLASbeforeNumPy, outputrootlock/duplicatetaskkey rejection, workerfailurecancelnew+joinrunning, existingchecksumfailclosedresume. Currentwriters exclusivecreate are NOT concurrent-outputlocks. Preserve full42400mainbatches +all100MCsensitivity; evaluate/exportserial minimalchange. Required serial-parallel hashparity/reversecompletion/duplicate/partialfailure/resume/freeze-barrier tests and actualG0 probe remain NOT_RUN. This is feasibilityreview, not parallelqualification.

Parallel integration initialfocused56753 failedcollection exit4: new module-level pytestparametrize used pytest previously imported only inside localtests, NameError. Failed log/XML preserved parallel_analysis_revision_001/parallel_slice_tests.*. Added module-level import; rerun required before relying on parallel integration. Noactualanalysis/V0 evaluated.

Parallel integration retry81258:58PASS/1FAIL, existingCLI adaptertest fake omittednewworkers kwarg; productionCLI correctly forwards it. Updatedfakeexplicitworkers parameter and assertions for bothdefault1/explicit2; no criteria relaxed. Failedretry log/XML preserved.

Parallel slice84854 terminal0:60PASS15.88s/existingAxes3D warning1 afterexplicitCLI fakefix. Realspawnhelper13tests +mainfullmatrix/sensitivityallrep/CLIworkers/freezeonfailure/streamedexport/auditor fixtures. Parentreviewed helper env-before-spawn, bounded queue, duplicate keys, outputlock, cancel+join andadded contextlib.closing inbothconsumerloops. Sourcehashbound parallel_slice_result.json; actualG0 hashparity/resourceprobe remains NOT_RUN. Next independent storage slice changes only immutablederivedcopy ownership; preserve this passashistorical pre-storage.

Combinedparallel/storage slice57600 terminal0:70PASS16.25s/existingAxes3Dwarning1. Added owned-root hardlinkoptin at8actualanalysiscopy sites, helperrejectsraw/outside/crossdevice and verifiesbefore/after; legacyexternalinputs stillindependentbytecopy. _analysis_hashes bindsbothhelpers, executionworkers andparentthreadenv inmanifest/worker count inanalysisversion. Source settings/scientificmatrix unchanged. ADR0001/0002 proposed pendingactualgates. ActualG0-only P0/P4trial0 full8yaw x2scopes withunchangedresume nowrunning: serial65628/spawn210911, logs+driver g0_parallel_probe_001; BLAS1, two outputroots, noV0 droneposition reads/evals (camera generator structuralcamera-pose checksall39 disclosed), familyRSS sampled0.25s/uniqueinodeallocatedbytes. Sourcefiles frozenforprobes; fullanalysis NOT_RUN.

Actualspawn2probe10911 terminal0:32G0 P0/P4trial0 full8yaw2scope cases, first170.13784s/resume10.22637s, entire scientificinventory unchanged afterresume. Sampledprocessfamily RSSpeak310194176B(0.25ssum/sharedpagescounted),4processes; selfpeak88196KiB; logical70717224B vsuniqueinodeallocated32137216B,758files566inodes. Serial65628stilllive; no serial/parallel parity orspeedup claimuntilfinished. Current44filebroadregression73110live. Independent integration review found onlyquoted~/outputrootlock normalization mismatch: facade locks expandedroot butpasses originalpath tobodyresolve; absoluteprobesunaffected. Keepallsourcefixeduntilhashboundprobes/tests finish, thennormalizeonce via lockyieldedroot andfocusednegativepathguard test. Full100MC/V0accuracy NOT_RUN.

MainG0serial65628 terminal0:32cases299.6818s/resume14.5777s. Exactcomparison PASS756scientificfileSHA256 entries acrossserial/spawn2, includinggeneratedinputs,methodresults,checksums,index/pathselectioninputs; bothunchangedresume. main_parity_comparison.json bindsbothsummaries; timingconcurrent/sharedhost, notisolatedscalability. Full44file73110 terminal0:338PASS181.51s, sourcehashesunchanged. ThenfixedONLY publicanalyze outputrootfacade to pass lock-yieldedcanonicalroot; normalized_root_delta_check ASTprovesallothermodulecodeunchanged vsfull338/probeepoch; negativefacadesecondlock guardincluded,71focusedPASS16.39s(session14321).

Actualsensitivity qualification started currentfixedsource:18993serial/3783spawn2, G0P0/P4trial0 all25OFATlevels50cases each+unchangedresume; P4fixedengineeringprobe path explicitlyNOTcampaignpathselection. Realdriver/metadata g0_parallel_probe_001/sensitivity_probe.py, outputssensitivity_workers_1/2. Calibrationonly; noV0 dronepositionerror/evaluation. Newsourcefreeze untilprobesendhashesverify. Pixel/enclosure90200ongoing. Full100MCmain/sensitivity/V0 NOT_RUN.

Sensitivityprobes18993/3783 terminal1 at first RTKinput: probeharness passed unnormalized absolute__file__path containing../.. to privatehelper; RTKgenerator returnsresolvedpath, relative_to(output_dir.parent) rejectedrepresentationmismatch. Publicproductionanalyze now passescanonical lockroot; no numerical/source-runtime bug implied. Bothworkersdrainedbyhelper; failedcaseinputs/logs retained sensitivity_workers_1/2, originaldriver copied sensitivity_probe_path_failure.py. Correctedharnessoutput to __file__.resolve().parent andassert lock-yieldedowner matches; rerunnew *_normalized_001 directories, nooverwrite/no rawchange/noV0read.

Rawsourceprovenance cadence_fix audit persisted final_raw_provenance_001:52/52runtime(name,SHA) variants matchpreservedZIPbytes;39selectedconfigs/reference/workbookchecks;8/8historicalenvreport exactSHA blobs (6reconstructedfromfullhistoryobject usingknownkeyorder,2verifiedcopies). Rootindependent_rehash rehashed8blobs+history/template,19archives/365matchingmembers,39effectiveconfigs PASS. Fiveearlycontroller_launch argvfiles absent; logicalcommands reconstructablefromfrozenrunner butNOTobservedargv, limitationretained; nofakefiles/no rawedits. Historicalmesh/dependencybyte limitations remainexplicit.

Oldscope sensitivityprobes43155/56488 terminal0:50all_dataOFATcases each+unchangedresume;780scientificfilehashes identical. Serial466.223s/spawn2200.813s; prototype ONLY. Capacity8probe45933 terminal0 main128cases+unchangedresume,236.294s/resume14.606s; sampled familyRSS937189376B,logical268462045B vsuniqueallocated122687488B;754overlappingcase/generated hashes identicalvs2workerprobe.

REQUIREMENT GAP FOUND BEFORE FULLANALYSIS/FREEZE/V0: actualspec reprocessing.matched_sample_policy isgeneric, instructionsec7requires all-data AND200pair comparison withoutOFATexception. Independentrequirementsreview confirmsOFATshouldcarryBOTHscopes; full_factorial=false restrictsfactorcrosses, notsamplepolicy. Earlier25kall_data-only OFAT plan superseded (notusernewscope): required50kcases ifselected≠P0,25kifP0, all100MC/all5reps; main42400/raw39unchanged. Existingoldscopeprobes remain engineeringprototypes, notfullcompliance. Rootanalyzer snapshot sensitivity_matched200_revision_001/analyze_campaign_before.py. Proposedminimalworker same(scenario,run,trial) owns shared_inputs andtrial/{all_data,matched_200}; matchedselection AFTERmodifiedpairing+RTKalignment; sensor/RTKrealizationsshared; <200explicitunmet. Evaluation/CSV/summarybaseline keyedbyscope. Offlineagentownsindependentresultauditor+tests only. Full100MCnotlaunched; V0positionerrorsunopened.

Independent dual-scope OFAT auditor slice completed:15PASS5.23s agenttests, onlyaudit_v12_analysis_results.py/test_v12_analysis_results_audit.py changed. Requires scenario/run/trial/scope matrix andtrial_NNN/scope paths, configs sample_scope, bundle/eval/methodrows data_scope, baseline_data_scope andcomparison.scope, crossscope sensorhash equality, existingmatchedselectionevidence, explicitcalibrationNOT_RUN andV0UNMET. SHA2defea94...; producerstilloldall_data-only untilrootimplementation. Thisauditorpass is notactualfullanalysisPASS; actualauditorNOT_RUN. Schema persistedcheckpoint.

Dualscope OFAT producerimplemented: sameworker owns shared_inputs plustrial/{all_data,matched_200}; sensor/pairs/RTK generation once, verifiedhardlinked snapshots; selectionaftermodifiedpairing+RTKalignment withexistingmatchedselector; percase/config/bundle/methodrow scope andsharedsensor hashes. Evaluationguard requirespersistedfreeze; scope-specificV0selection/Y0baseline, explicitMATCHED_200_UNMET emptyresultpayload+sixCSVaccountingrows, noaccuracyfabrication. CSV/summarygroupincludes scope. Initialcombined61464:46PASS/2fixturemismatches (oldtest expectedgenericNOT_RUN, missing requiredsyntheticbaseline data_scope); agentcorrectedestablishedstatus/fixture, addednegativewrongbaseline test. Failedlogsretained. Actualsourcefull100MC/V0NOT_RUN.

Dualscope retry33027:48PASS/1FAIL exposedCSVmissingbaseline_data_scope; addedCSVfield. AlsoalignedmatchedV0pair_summary denominator toexistingmain200/200 policy, retaining source_pair_summary separately; scopedavailabilitymustnotartificiallydivide200bynominalfullcapture. Addedmockboundaryassertions. Retry23491:47PASS/2fixtureerrors because syntheticpairsummary lackedrequired both_detected_science_pair_count; fixture nowdeclaresitsactualn, noalgorithm/gaterelaxation. Failedlogs/XMLretained.

Final pre-V0 integration slice: session16223 PASS100/9.74s (identity guard, paired attitude report, orchestration, dual-scope sensitivity, result auditor, capture summary). Extended coordinator fixture now carries all six A/B/C x RPY_IMU/YAW_ONLY rows with full identities. Added drift-during-calibration integration: persisted freeze allowed, but no V0 evaluation/admission/latest pointer when current source differs; session14778 PASS28/6.83s. These use synthetic evaluations, not real V0 scores. Existing Axes3D warning only.

Actual dual-scope G0 parity independently read back:100 cases each,1786 byte-identical scientific files, unchanged resume; serial493.003s/spawn2278.937s. Visibility completed39 under stated renderer/model assumptions:48154 images, minimum observed margin152px, conservative128.008px. Actual39 capture summary with lateral travel written capture_summary_39_with_flight_costs_preanalysis.json without positioning error disclosure.

Current final regression session27702 launched47 test files with archived source ZIP/hashes and log/XML under source_evidence/pre_v0_final_revision_001; terminal result pending. Capacity admission ADMITTED:198.90GB observed free vs167.01GB estimated requirement including1.5x measured fit projection,20GB evaluation,20GB exports/temp,25GB engineering reserve. Full scientific counts42400main/up-to50000OFAT,100MC unchanged; workers8 qualified G0 peak familyRSS937MB. This is a planning estimate with live disk monitoring required, not final measured size. Full actual100MC analysis still not launched at this entry.

Final current-source regression27702 terminal0:403PASS/127.09s,47 files, archive hashes unchanged. ADR0001/0002 accepted with actual G0 parity/resource and recorded capacity evidence; historical pending language superseded by qualification section. FULL actual100MC analysis launched sole session60188,8workers, same canonical campaignroot. command/PIDs/log/terminal receipt under source_evidence/full_analysis_execution_001. No actual V0 accuracy opened before prescribed freeze/admission. Source files now fixed for full execution; subsequent changes require disclosed versioning rather than silent mutation.

CORRECTION: full60188 terminal2 before fitting; first actual analyzer admission exposed stale count-field adapter. Actual QC G0-P0-R01/a02 VALID_WITH_WARNINGS, infrastructure/protocol errors empty, counted_in_campaign=true but counted_in_39=false by intentional workbook-source contract. Old synthetic workbook QC mocked legacytrue and hid mismatch. Fixed _status_slots to require counted_in_campaign for verified workbook sources, counted_in_39 for legacy; strictstatus/fullQC unchanged. Added workbooktrue/false tests.45focusedPASS7.17s; source_admission_delta.json proves every other analyzer AST unchanged vs403pass epoch. Actual corrected analyzer _status_slots all39PASS43.096s. OriginalfailureQC/log/receipt and correctedsourcearchive retained full_analysis_execution_001. New sole full100MC execution session86757 under full_analysis_execution_002; nofit/V0results from failedlaunch and no rawmutation.

Live full86757 PID370079 independently confirmed with8 spawned CPU-active workers. analysis_versions/analysis_9cfd7c6cd10b, 67persisted primary calibration bundles at observation1792271769207871895ns; freeze absent as expected, noV0evaluation. Disk198.83GBfree. Added progress_samples.jsonl, not source identity mutation.

Raw handoff inventory actualPASS:39selected firstvalid==latest;54attempt directories retained(39VALID_WITH_WARNINGS/15INFRASTRUCTURE_FAILED);all15required paths per selectedrun exist;all188380 left/right observation rows checked all16requiredfields andrun/attempt identity,0problems. This complements fullQC rather than replacing rawchecksum/content checks. Standaloneaudit script+report final_raw_handoff_inventory_001. Interim15item hashed evidenceindex final_handoff_evidence_index_001 explicitly lists pendingfullanalysis/freeze/V0/auditor/exports/finalhandoff andlimitations; not finalcompletionclaim.

Fullanalysis86757/PID370079 verifiedlive again,8CPU-active calibrationworkers. Added read-only monitoring session62440, preserved monitor_script.py/resource_monitor.jsonl under full_analysis_execution_002:30s processfamily CPU/RSS/output/tmpfree/sourceidentity;300s calibration-bundle metadata census. PID starttick check prevents PIDreuse confusion; stops on actual terminal receipt, never silently restarts analysis. Initial137persistedprimarybundles,0sensitivity,1.065GBsampledfamilyRSS,198.759GBfree, allsourceidentity hashes stable,freeze/admission absent as expected. Monitor reports inspection-required below25GB engineeringreserve or sourceidentity drift; it does not weaken scientific gates or terminate writers midway. Final scope unchanged.

Handoff follow-through: existing output-format contract still requires a current-campaign exactly8sheet 전체_실험_요약.xlsx, distinct from analyzer's detailed existing_yaw/gps_yaw90 result workbooks. Currentcampaign 요약_결과 contains only preanalysis JSONsnapshots; no final summaryXLSX yet. Re-read original preserved outputdesign; copied exactbytes andrecordedSHA+sheetnames+pendingtask in final_handoff_evidence_index_001. Populate from current39/costsummary plus actual auditedreleasedanalysis only, then reconcile values and visuallyreviewall8sheets. Historicalsource dataset/workbook mustremainreadonly and its oldperformance outcomes mustnot substitute currentresults.

Summaryworkbook preparation: bundlednode24.19.0 artifact-tool Workbook/SpreadsheetFile imports verified, no workbookcreated/exported/noV0resultsread. source_evidence/summary_workbook_preparation_001 records runtimecheck and8sheet sourcemap/releasegates. Capture summary lacks actualheading aggregate; raw flight_state exposes px4_heading_rad plusphase/simulationidentity. Final extractor mustcompute explicit sciencePX4estimatedheading-vs-command and/or frame-verified GTphysicalheading afterrelease, label distinction andmissingness. This requiredfield is pending, not fabricated or assumed covered by positiontracking metrics.

Closed heading-aggregate implementation gap without changing frozen analysis source: standalone final_report_tools_001/heading_metrics.py usesstdlibrary only, separates PX4estimatedheading vs Gazeboreference+X projectedworldheading, wrappedclockwisefromnorth residual vs recordedconstantreference. Exactscienceepoch/events boundaries; duplicate/nonfinite/unknownframe/vertical-forward cases rejected or explicitundefined, P95 linearinterpolation, noaccuracythreshold. CLI gates actualfullresultauditPASS before opening anyanalysis summary/rawV0, checksfreeze/admission/sourceidentity, reportsfirstvalid all39.5testsPASS inclcardinals/antipodalquaternions/wrap/P95/undefinedheading andfailedrelease beforeanalysisread; tests.log+checksums preserved. RealG0P0R01/a02 developmentreadback1315sciencePX4/1315GTsamples,0undefined, noV0read/noestimatedstereoposition. Full39headingexecution remainsPENDING untilrelease.

Live intermediate artifact readback calibration_sample_readback_001:20available rig/yaw/scope strata,1completed checksum-bearing case each,180method/variant rows. Allmandatory calibration resultfields present andalllocalchecksummed generated files verified;0errors/noevaluationartifacts opened. This is explicitly sampled structural/hash evidence, not full42400matrix audit or numerical correctness certification. Full86757 andmonitor62440 remainlive; continuedCPU/artifactincrease observed after45s wait, no restart/sourcechange.

Queued authorized final verification continuation session62170, livepostanalysis supervisor source/plan/events in post_analysis_verification_001. It waits actual full_analysis_execution_002 terminalreceipt; nonzero stopswithnostages/noautorerun. Onactual0 checks expectedanalysis_latest version andpinned auditor/heading/summarizer sourceSHAs, then sequentially executes actualfullresultsauditor -> onlyPASSallows heading39 -> finalsummaryJSON. Exclusive reports/logs; anyfailure stops nextstages. Currentstate WAITING, these stages NOT_RUN, notclaimedcomplete. Final8sheetworkbook and finalrequirementaudit remain explicitpending. Fullanalysis86757 untouched; no sourceidentity mutation or prematureV0 reads.

Preserved frozen_analysis_source_snapshot_001:13algorithm sourcefiles currentbytes match running analysismanifest; ZIPcontains exactsources+capturedmanifest; independent ZIPmember rehash all13PASS andmanifest byteequal.147716byte scopedbinary trackedsource diff vsactualHEAD6810b228e7783c6420fd456e877c978e56a66872, branch/scopedstatus andarchivehashes recorded. Untrackedfiles fully preservedZIP; patch explicitlydoesnot claim them or unrelatedGUIchanges. Separate historicalruntime variants remain final_raw_provenance_001. Readonlysource capture; noanalysiscode/primaryinputchanges.

Prepared8sheet summary_artifact builder source summary_workbook_preparation_001/build_summary.mjs; node--checkPASS only, notexecuted/NOTqualifiedruntime/export/render. Usesbundledartifact-tool, auditedrelease gate, exact8sheetorder, typedrectangulartables, nativecell-boundcharts, metadata+PNGQA/export, nooverwrite. NotoSansCJK KR installedfont verified. Dataextractor and actualreleasedpayload stillpending; skill operationmarker mustrun immediatelybefore first actualauthoring. Frozenanalysis code untouched. This is implementationprogress only, not deliveredXLSX.

Prepared release-gated extract_summary_payload.py outsidefrozenanalysis;4meaningful synthetic testsPASS (auditFAIL beforeanalysisread, zero-vs-missing, changedcasehash, Euclideandistance/stale/missing/epoch behavior andnumericformats). Per-casecal/evalSHAchecked againstcompletedindex; exactFraction available-case means, separatefailed/available/physicalrun/MCcounts; allattempts retained and8prescribedtables/nativechartpayload. RecordedscienceGT-ref positionaltracking includesmaxGTage/stale>40ms/missingcounts andexacteventepoch bounds, labelledrecordedstatecomparison notinterpolatedcontinuous truth. ActualG0P0R01/a02 developmentreadback1315states,0stale/0missing/maxGTage24ms; noV0read. Fullreleasedpayload/XLSXstillNOT_RUN; no frozenanalyzer/supervisorpinnedsource changes. Preparationplan/sourcehashes updated.

Finalreport preparedextractor now6testsPASS: explicitnonconvergence denominatorattempted versusfailure/notrun denominatorallcalcases, all-unavailableGPS retained, controlsratiosnull. Addedcase-paired RPY/YAWsupplementallgroups to04andmainoriginal-yaw all_data outcomechartto05; fullpairedtablekeepsGPS/matched/sensitivityseparate. Testensureschartpopulationdoesnotmixthosegroups; improved/regressed/nochange/notcomparable counts copiedfromfrozenpairedreport.3nativecharts nowplanned. BuildernodecheckPASS aftersupplementtable/render/wrappingchanges; realWorkbookcreate/export/visualQAstillNOT_RUN. Prepsourcehashes refreshed; currentanalysis/supervisorpinnedsourcesunchanged.

Earlyactual full100MC coveragecensus primary_matrix_census_001: checksum-bearingcase identities fortrial000/001/002 eachmatchall424expectedrawcalibration-slot/yaw/scope combinations,0missing/0unexpected. Latertrial003/004 inprogressatobservation;1688checksum-bearingcases total. Filesystempresence/identity only, notfullchecksumbyteaudit or scientificfitcertification; realV0 artifacts unopened. Full86757 remainslive throughsubsequent45sverifiedwait, sourcesfixed, no rerun.

Observations-report denominatorfollowthrough: actualpostprocesscounts have source_frames/inferred/skipped_rate_limit/detected/not_detected/inference_error. Final03payload nowseparately showspercamera attemptcounts/attemptcoverage, attemptconditional detectionrate, skip/errorcounts, originalfps andsourceconditional detectionrates; counterequations checked, zeroattempt yieldsnullconditionalrate notfakezero. Corrected warning display adapter because summarize_campaign.qc_warnings isalreadyjoinedstring; nolongercharacterjoin.8extractortestsPASS inclbothnewcases. Noactualfinalpayload/XLSX/noV0read, originalanalyzerunchanged.

Prepared independentread-only savedXLSX verifier verify_saved_summary.py (stdlibOpenXML, noauthoring).3helpertestsPASS forshared/inlineKoreantext/errorcells, exactID/count/type/nullvs0 andfloat tolerance, contiguous/noncontiguouschartreference normalization. Actualverifier checks matchingactualauditSHA/analysisID, all8sheetnames/order andeverypayload/table/supplementvalue, chartpartcount+formula-boundseriescategories/values anddrawingattachmentto05. ActualfinalXLSX NOT_CREATED/verificationNOT_RUN; sourceextractor/runtime/render/visualreview andfinalrequirementaudit remainnecessary. Sourceprepplan/hashes updated; frozenanalysis/supervisorpinnedsources unchanged.

Summaryreport completenessfollowthrough: final04preparedpayload nowincludes existinglateral-east/forward-north/vertical-up RMSE andsignedbias available-case means, alongside3D/RPY/covariancecondition/failureavailability. No numericalresultsrecomputed orV0read; extractor8testsPASS withaxis/bias/zero/missing assertions; ASTheader cardinality37=17fixed+20metrics checked. AddedhorizontalQA panels coveringallwide tracking/quality/comparisoncolumns inadditionto8sheet andpairedsection previews; buildernodecheckPASS. ActualXLSXruntime/export/render/reopen stillNOT_RUN. Prepplan/sourcehashesupdated, frozenanalyzerunchanged.

Closedprepared98freeze-time provenance gap: extractor9testsPASS withUTCnanosecond exacttext andmonitorlast-absent/first-present observation semantics. DeterministicfreezeJSONhas noUTCfield; reportwilllabel observedfilesystemmtime andmonitorbounds separately, neverinvent anexacttrustedfreezeinstant.98columns source/hash/UTC/description staytyped; addedfrozeninput/spec/env/model/world/visibility evidencehashes andsame39runYOLOmodelSHAcheck from observation summaries. Actualreleasedextraction/XLSXstillNOT_RUN, noV0read/noanalyzerchanges. Prepplan/digests/resumecheckpointupdated.
