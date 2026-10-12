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
