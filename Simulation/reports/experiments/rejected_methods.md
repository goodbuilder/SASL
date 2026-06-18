# Rejected Or Paused Methods

This file records branches that should not be rerun without a material change
in inputs, observation model, gate, or method.

## Rejected For Current Gate

- Bbox non-center geometry-only anchors
  - Reason: no all-report strict improvement; lateral regressions remain.
  - Revisit only if the observation model changes.
- Bbox size-derived range from width/height/area
  - Reason: bbox extent does not consistently represent physical target size.
  - Revisit only with reliable physical extent estimation.
- KNN/local residual lookup with current features
  - Reason: conservative variants still caused small lateral regressions.
  - Revisit only with stronger features or a changed gate.
- Tracking/Kalman as standalone correction
  - Reason: temporal filtering did not satisfy RDF all-axis gates.
  - Revisit only after a spatial correction has already passed validation.
- Current extrinsic global-bias implementations as final methods
  - Reason: useful Forward baselines but not stable all-axis solutions.
  - Revisit if the extrinsic fitting target or calibration setup changes.
- `epipolar_forward_projection_centerrot_forward_protected_rd`
  - Reason: Forward improves strongly, but Right and Down are protected no-op
    axes with exactly zero improvement; strict RDF pass is `0/90`.
  - Revisit only as a Forward-only baseline, or with a real Right/Down
    correction source.
- Single universal exported d6/d7/d12 runtime profile across all synthetic
  stress scenarios
  - Reason: EXP-20260615-188 fixed-profile scenario transfer failed `241/300`;
    intrinsic scale stress failed `0/50`, disparity-noise stress failed `49/50`,
    and bbox-outlier stress failed `42/50` with target max abs error up to
    `19.585m`.
  - Revisit only with scenario/catalog selection, runtime measurement guard,
    or separate profiles for intrinsic/noise/outlier stress. The same profile
    remains valid for nominal/yaw-style deployment and fresh nominal transfer.
- Bbox-outlier single-frame profile retuning without extra observations
  - Reason: EXP-20260615-190 Huber sweep (`1/2/4/8`) and EXP-20260615-191
    Forward residual clamp sweep (`4/6/10/20m`) both stayed at `39/50` fresh
    bbox-outlier transfer passes. Corrected mean range RMSE improved versus raw
    (`~2.659m -> ~1.320m`), but the 1852m target max abs error remained
    `~18.84m`, above the `10m` pass limit.
  - Additional probe: EXP-20260615-193 existing inverse-depth and
    left-disparity geometry runtime profiles also stayed at `39/50` on the
    same fresh bbox-outlier transfer audit. Temporal median/local-linear and
    Savitzky-Golay smoothing probes did not exceed `39/50` and often worsened
    aggregate RMSE.
  - Interpretation: in the current synthetic `bbox_outlier_stress`, some
    disparity outliers shift both measured disparity and raw Forward
    consistently, so `raw RDF + measured disparity` single-frame features do
    not identify the bad sample. Revisit with repeated observations, tracker
    association confidence, image-space quality signals, multi-frame fusion
    with a validated motion model, or a less adversarial outlier model.
- Causal alpha-beta motion fusion over corrected RDF as a standalone
  bbox-outlier fix
  - Reason: EXP-20260615-194 runtime-safe causal RDF alpha-beta innovation
    clipping improved the bbox-outlier transfer pass count from `39/50` to
    `47/50`, but still failed the strict 1852m target gate with target max abs
    error `13.786m`. EXP-20260615-196 fixed the forward-only axis pass-through
    contract and reached the same `47/50` with target max `13.788m`.
    EXP-20260615-197 stronger forward smoothing reduced pass count to `45/50`
    or worse and did not reduce target max below `13.867m`.
  - Revisit only with a richer motion model, explicit association confidence,
    image-space quality features, or repeated independent stereo observations.
    The current causal filter is useful as a runtime stabilizer candidate, but
    it is not sufficient as the final bbox-outlier correction method.
- Causal image-space alpha-beta prefiltering as a standalone bbox-outlier fix
  - Reason: EXP-20260617-198 showed that aggressive filtering of
    `left_u_px`, `left_v_px`, and `measured_disparity_px` can catastrophically
    distort the normal range sweep when disparity innovations are clipped too
    tightly. EXP-20260617-199 added wide-clip sanity checks: the no-op-like
    `alpha=1,beta=0,uv=999,disparity=999` setting reproduced the base profile
    (`39/50`, target max `18.836m`), while the best guarded image prefilter
    reached only `46/50` with target max `15.397m`.
  - Interpretation: image-space prefiltering can reduce some bbox/disparity
    outlier impact, but direct alpha-beta filtering over a strongly nonlinear
    disparity-vs-range trajectory is not enough for the strict 1852m target
    gate. Revisit with inverse-depth-domain filtering, explicit detector
    confidence, or model-based prediction in calibrated 3D rather than raw
    disparity pixels.
- Causal inverse-depth-domain alpha-beta prefiltering as a standalone
  bbox-outlier fix
  - Reason: EXP-20260617-200 filtered disparity as inverse depth
    `disparity/(fx*baseline)` before re-triangulation. The best setting was
    effectively no inverse-depth clipping (`id999`) with mild image-center
    smoothing: `46/50`, target max abs error `15.397m`. Real inverse-depth
    clipping did not improve the strict target gate.
  - Interpretation: for the current synthetic bbox-outlier model, the remaining
    failures are not solved by smoothing disparity or inverse depth alone. The
    next useful branch needs detector confidence/association quality, repeated
    independent observations, or a model that can explicitly reject false
    image associations.
- Repeated observation fusion when burst observations share common-mode
  image/disparity outliers
  - Reason: EXP-20260617-210 added a synthetic common noise/outlier fraction
    inside the same timestamp/burst. With `n=31 trimmed_mean`, common fraction
    `0` passed `50/50`, but common fraction `0.25` dropped to `46/50` with
    target max abs error `16.586m`, common fraction `0.5` dropped to `42/50`
    with target max `25.425m`, and common fraction `1.0` dropped to `38/50`.
  - Interpretation: repeated-observation fusion is a valid bbox-outlier fix
    only when the observations are meaningfully independent. Repeating the same
    correlated detector/crop failure does not satisfy the runtime condition.
    Revisit with detector augmentation diversity, tracker association
    confidence, epipolar/size/kinematic gates, or a separate common-mode
    outlier detector before applying high-count fusion to real StereoDistance.
- Small-drone observation fusion without a calibration-time correction profile
  - Reason: EXP-20260617-213 tested small-drone close and mid camera geometry
    with fresh 50-seed validation per scenario. `fusion_only` failed every
    tested branch: low-latency `1x3 common0` passed only `17/50` close and
    `12/50` mid, while `1x3 common1` dropped to `6/50` close and `5/50` mid.
  - Interpretation: robust observation fusion reduces random/outlier noise but
    does not remove fixed small-drone calibration bias. Revisit only with a
    calibration-time Sim3/affine profile, an explicit intrinsics/baseline
    recalibration, or a materially different runtime observable that estimates
    the fixed bias without truth.
- `target_guard` representative profile from the runtime-observation stress
  sweep as a default single profile
  - Reason: `Simulation/results/rtk_stereo_runtime_observation_stress_EXP_20260617`
    selected `sim3_ridge_raw_rdf_residual_field_deg2_alpha100_cap4m` for the
    target-guard policy, but transfer replay passed only `5/6`; on
    `bbox_quantization_stress` it had one no-improvement failure with minimum
    validation RMSE improvement `-0.711103m`.
  - Interpretation: the target-window error objective alone can overfit the
    calibration stress shape and should not drive default deployment selection.
    Keep it as a diagnostic or scenario-catalog fallback only after explicit
    scenario labeling; do not replace the validation-balanced profile with it.
- Unconditional replacement of the current active profile with
  `target_weighted_huber_joint_disparity_ray_rotation_nls_sigma060`
  - Reason: the stress-selected profile passed deployment smoke and won
    corrected range RMSE on `5/6` runtime-observation stress splits, but
    `profile_compare_all` also shows the existing active
    `sim3_binned_residual_disparity_bins11` wins one bbox-quantization split.
    The earlier nominal smoke comparison also favors the existing active
    profile (`0.885m` corrected range RMSE versus `1.336m` for the stress
    profile).
  - Interpretation: the stress profile is valid as a scenario-labeled
    deployment candidate. It should not become the unconditional default until
    recorded calibration CSV or truth-free deployment metadata proves that the
    live run belongs to the observation-stress regime. Runtime switching must
    not inspect PX4/RTK truth or validation errors.
- Selector-only portfolio over the currently deployed all-scenario
  observation-fusion aggregate tiers
  - Reason: EXP-20260618-228 reran the offline tier portfolio oracle after
    excluding `single_source` fallback tiers from the default comparison. The
    aggregate-only oracle covered 4 comparable scenarios and selected
    `fresh_common_mode_b9x3_range_quality` for bbox outlier,
    `fresh_common_mode_b16x3_target_tail_upper_bound` for combined/intrinsic
    stress, and the default `fresh_common_mode_b7x3_aggregate_best` for
    disparity noise. The default target worst and oracle target worst were
    identical at `5.063616m`.
  - Interpretation: choosing among the currently deployed all-scenario
    aggregate observation-fusion tiers is not enough to reduce the worst-case
    1852m target-tail error. Future work should introduce a new
    disparity-noise-specific observation/residual model or a truth-free
    runtime feature selector with evidence that it improves the disparity
    stress case. Runtime selection must remain based on image/disparity/profile
    metadata only, not PX4/RTK truth or validation errors.
- `b7x3` low-trim/mode-only search as a disparity-noise breakthrough
  - Reason: EXP-20260618-229 matched EXP-218's scenario-loso profile and seed
    range, then swept `b7x3` trim `0.05/0.10/0.15` with inner/outer
    `median/trimmed_mean/mean` on `disparity_noise_stress`. The best rows tied
    the existing `b7x3 trim0.1` target max `5.018522m` and range mean
    `1.997943m`; median outer variants were worse. Several low-trim/mean
    variants collapse to the same fused value under the discrete `7x3`
    observation geometry.
  - Interpretation: the current disparity-noise bottleneck is not solved by
    only changing the `b7x3` robust statistic. The next branch should change the
    observable/model itself, for example inverse-depth residual modeling,
    disparity innovation gating, temporal motion prediction, or a truth-free
    runtime feature selector that can identify when a disparity-specific tier is
    warranted.
- Causal alpha-beta motion fusion as a disparity-noise fix
  - Reason: EXP-20260618-230 applied causal alpha-beta innovation clipping to
    corrected stereo-center RDF on `disparity_noise_stress`. It remained
    STRICT_PASS but its best target max `8.555299m` and range mean `2.298741m`
    were worse than b7x3 observation fusion. EXP-20260618-231 then combined
    b7x3 image/disparity multi-burst fusion with the same motion gate. The best
    row was effectively no additional gate (`alpha=1.0`, `clip=40m`) and tied
    observation-only target max `5.018522m`; stronger gates regressed target max
    above 20m.
  - Interpretation: temporal smoothing over corrected RDF can hide or lag
    legitimate long-range trajectory changes and does not remove the current
    disparity-noise tail. Keep motion fusion as a separate runtime stabilizer
    candidate only when operator display smoothness is the objective; do not use
    it as the final 1852m target-tail correction method for this scenario.
- Single unconditional pooled disparity residual profile across all observation
  stress scenarios
  - Reason: EXP-20260618-236 used
    `run_runtime_pooled_disparity_residual_profile.py` to fit one
    `DisparityResidualRuntimeProfile` from the pooled train splits of
    bbox-outlier, combined-sensor, disparity-noise, and intrinsic-scale stress
    scenarios. The best single profile was
    `ray_scale log_disparity degree3 ridge1 clip0.5`, but it passed only
    `60/80` validation splits. It reduced target worst
    `5.018522m -> 4.691842m` and average range RMSE
    `1.530827m -> 1.370467m`, but failed every intrinsic-scale validation split
    under the raw-improvement gate and worsened disparity-noise mean range RMSE
    `1.997943m -> 2.097912m`.
  - Interpretation: the residual shape is scenario dependent. A single global
    polynomial is too blunt for unconditional runtime deployment. Use
    scenario-labeled profiles or a future truth-free runtime selector instead;
    do not select among these profiles using PX4/RTK truth, validation metrics,
    or target-window labels at runtime.
- Feature-domain residual-profile guard using only measured-disparity z-score
  and residual correction magnitude
  - Reason: EXP-20260618-240 evaluated
    `evaluate_disparity_residual_feature_guard.py` on the EXP-238 residual
    catalog across bbox-outlier, combined-sensor, disparity-noise, and
    intrinsic-scale stress scenarios. The sweep covered 20 seeds/scenario,
    four profile labels, z p95 thresholds `2/3/4/5/6`, and correction p95
    thresholds `0.01/0.03/0.05/0.1/0.2/0.5`. No threshold pair produced a
    viable guard. The safest bbox row blocked all wrong-profile applications
    but also applied residual zero times (`apply_count=0`), so it is only
    residual-disabled fallback, not selection. Combined and disparity profiles
    kept label-matched `20/20` but still applied to `33/60` wrong-profile
    trials. Intrinsic applied to `60/60` wrong-profile trials.
  - Interpretation: profile feature-domain overlap is too large for this
    two-signal guard to replace scenario labels. Keep the EXP-238 selector as
    explicit deployment/operator label driven. Automatic selection needs richer
    truth-free observables such as image-quality statistics, detector/tracker
    confidence, epipolar consistency, temporal innovation features, or
    calibration metadata. Runtime selection must still exclude PX4/RTK/NED
    truth, validation RMSE, target-window labels, true disparity, and true
    image features.

## Paused / Under-Evidenced

- `EXP-20260618-259-latest-telemetry-residual-preflight-probe`
  - Reason: latest telemetry discovery selected
    `Simulation/bags/operator_20260529_221251_lite/telemetry.csv`, but this
    older recorder schema does not contain the public FRD raw/corrected fields
    required by the residual preflight bridge:
    `raw_forward_m/raw_right_m/raw_down_m` and
    `corrected_forward_m/corrected_right_m/corrected_down_m`.
  - Gate result: `run_operator_telemetry_residual_preflight.py` correctly
    returned `FAIL`, `apply_residual_allowed=false`, `operator_launch_status=SKIPPED`,
    and `runtime_truth_used=false`. No residual profile was recommended or
    passed to the operator launch checker.
  - Interpretation: this is a schema readiness rejection, not a correction
    method rejection. Re-record operator telemetry with the EXP-251+ public
    FRD raw/corrected fields before enabling residual preflight on real logs.
    Calibration/evaluation truth remains offline-only; runtime selector inputs
    must remain truth-free image/disparity and stereo-center RDF fields.

- `EXP-20260618-260-operator-telemetry-schema-audit-gate`
  - Reason: this is a live-readiness gate, not a correction method. It adds
    a schema audit before residual selector replay so old operator telemetry
    cannot leave stale residual decisions behind.
  - Gate result: latest telemetry
    `Simulation/bags/operator_20260529_221251_lite/telemetry.csv` failed the
    audit with `9384` input rows, `0` valid rows, and missing public FRD
    raw/corrected groups: `raw_forward_m/raw_right_m/raw_down_m` and
    `observation_forward_m/observation_right_m/observation_down_m`.
    The selector decision artifact was overwritten as `SKIPPED` with
    `apply_residual_allowed=false`; operator launch stayed `SKIPPED`; runtime
    truth remained `false`.
  - Interpretation: EXP-260 makes the EXP-259 rejection easier to diagnose
    and safer to rerun in an existing output directory. Re-record operator
    telemetry with the EXP-251+ public FRD schema before live residual
    enablement. Source telemetry remains public FRD, selector observations are
    stereo-center RDF, and public output remains FRD+range.

- `EXP-20260618-261-operator-recorder-current-schema-lock`
  - Reason: this is a recorder/schema readiness fix rather than a correction
    method. The old bags remain unusable for residual preflight, but future
    `run_operator_view.sh --record-lite` telemetry must expose the fields that
    EXP-260 audits.
  - Gate result: `operator_recorder.py` now declares
    `stereodistance_public_frd:forward,right,down,range`, keeps raw/estimated
    and corrected public FRD fields in the CSV schema, and records
    `/stereo/profile_status` JSON metadata after adding the missing `json`
    import. Unit coverage writes a synthetic recorder row and verifies it
    passes `audit_operator_telemetry_residual_preflight_schema.py`.
  - Interpretation: this does not rescue old telemetry. It makes the next
    operator recording compatible with the residual preflight contract:
    public FRD telemetry source, stereo-center RDF selector observation, no
    runtime PX4/RTK/NED truth input, public output FRD+range.

## Passed Runtime Readiness Evidence

- `EXP-20260618-262-current-schema-operator-preflight-smoke`
  - Result: `PASS`.
  - Evidence: `Simulation/scripts/run_operator_current_schema_preflight_smoke.py`
    generated a `stereodistance_public_frd:forward,right,down,range` telemetry
    CSV, then ran residual preflight and readiness validation against the
    deployed feature selector model and residual catalog.
  - Metrics: schema audit valid rows `5/5`, selected label
    `disparity_noise_stress`, distance margin min `0.9253968578`,
    `apply_residual_allowed=true`, operator launch dry-run `PASS`, readiness
    `PASS`, runtime truth `false`.
  - Interpretation: this does not validate a real flight log, but it proves the
    EXP-261 recorder schema can travel through the complete truth-free residual
    preflight path. Next step remains a fresh `run_operator_view.sh
    --record-lite` recording followed by the latest-telemetry preflight.

- `EXP-20260618-263-latest-telemetry-residual-live-gate`
  - Result: `FAIL` on the current available bags.
  - Evidence: `Simulation/scripts/run_operator_latest_telemetry_residual_gate.py`
    selected `Simulation/bags/operator_20260529_221251_lite/telemetry.csv`,
    ran residual preflight, and skipped readiness because preflight did not
    pass.
  - Metrics: preflight `FAIL`, readiness `SKIPPED`, `apply_residual_allowed=false`,
    operator launch `SKIPPED`, runtime truth `false`, missing groups
    `raw_forward_m/raw_right_m/raw_down_m` and
    `observation_forward_m/observation_right_m/observation_down_m`.
  - Interpretation: this is the expected live gate rejection for old telemetry.
    The wrapper is now the one-command check to rerun after a fresh
    `run_operator_view.sh --record-lite` recording with the EXP-261 public
    FRD+range schema.

- `EXP-20260618-264-operator-recording-preflight-readiness`
  - Result: `PASS`.
  - Evidence: `Simulation/scripts/check_operator_recording_preflight_readiness.py`
    checks the static recording path before a fresh run: `run_operator_view.sh`
    supports `--record-lite`, calls `stereo_ground_sim.operator_recorder`, and
    `setup.py` declares the `operator_recorder` console entry point.
  - Metrics: recorder axis contract
    `stereodistance_public_frd:forward,right,down,range`, required raw,
    estimated, corrected FRD fields present, image center/disparity fields
    present, ambiguous x/y/z recorder fields `0`, runtime truth `false`.
  - Interpretation: the old bags still fail EXP-263, but the next recording
    path is schema-ready for residual preflight.

- `EXP-20260618-265-operator-residual-enablement-suite`
  - Result: overall `FAIL`, with partial gates carrying useful evidence.
  - Evidence: `Simulation/scripts/run_operator_residual_enablement_suite.py`
    runs recording-path readiness, current-schema preflight smoke, and latest
    real-telemetry live gate in one command.
  - Metrics: recording readiness `PASS`, current-schema smoke `PASS`, latest
    gate `FAIL`, selected latest telemetry
    `Simulation/bags/operator_20260529_221251_lite/telemetry.csv`,
    `apply_residual_allowed=false`, operator launch `SKIPPED`, runtime truth
    `false`.
  - Interpretation: software and recorder paths are ready. The only remaining
    live-enable blocker shown by the suite is the lack of a fresh current-schema
    operator telemetry recording.

- `EXP-20260618-266-record-lite-then-enablement-suite-wrapper`
  - Result: `PASS` in `PRINT_ONLY` mode.
  - Evidence: `Simulation/scripts/run_operator_record_lite_and_residual_suite.py`
    builds the `run_operator_view_with_calibration_profile.sh -- --record-lite`
    command and the follow-up residual enablement suite args without launching
    ROS/Gazebo/PX4.
  - Metrics: record status `PRINT_ONLY`, suite status `PRINT_ONLY`, runtime
    truth `false`, output command includes `--record-lite` and the configured
    active runtime profile.
  - Interpretation: this is an operational wrapper, not flight evidence. Use it
    without `--print-only` when ready to create the fresh current-schema
    telemetry that EXP-265 still needs.

- `EXP-20260618-267-record-lite-fresh-telemetry-guard`
  - Result: `PASS` in `PRINT_ONLY` mode, with additional guard fields recorded.
  - Evidence: `Simulation/scripts/run_operator_record_lite_and_residual_suite.py`
    now records `record_telemetry_csv`, `record_telemetry_exists`, and
    `suite_used_record_telemetry`.
  - Metrics: print-only evidence has
    `record_telemetry_csv=Simulation/bags/operator_next_current_schema_lite/telemetry.csv`,
    `record_telemetry_exists=false`, `suite_used_record_telemetry=false`,
    runtime truth `false`.
  - Interpretation: in real record mode, the wrapper now fails if the expected
    fresh `record_dir/telemetry.csv` is missing or if the suite selects a
    different latest telemetry file. This prevents silently reusing the old
    schema bags after a failed/empty recording attempt.

- `EXP-20260618-268-record-lite-synthetic-enablement-suite`
  - Result: `PASS` in non-print synthetic-record mode.
  - Evidence:
    `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_disparity_residual_probe/scenario_residual_catalog/feature_selector/operator_record_lite_synthetic_enablement_suite/operator_record_lite_and_residual_suite_summary.txt`.
  - Metrics: `record_status=SYNTHETIC`, `record_telemetry_exists=true`,
    `record_telemetry_fresh=true`, `suite_used_record_telemetry=true`, suite
    status `PASS`, latest gate `PASS`, selected residual label
    `disparity_noise_stress`, runtime truth `false`.
  - Interpretation: this proves the current-schema public FRD+range telemetry
    can pass the non-print record wrapper, freshness guard, latest telemetry
    selection, residual preflight, readiness check, and operator launch dry-run
    without PX4/RTK/NED truth. It is still synthetic software-path evidence;
    the next promotion step is the same wrapper with real `--record-lite`
    telemetry from Gazebo/PX4 or the live operator stack.

- `EXP-20260618-269-installed-recorder-overlay-schema-gate`
  - Result: `PASS` after rebuilding the ROS overlay; before-build probe was
    `FAIL`.
  - Evidence before build:
    `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_disparity_residual_probe/scenario_residual_catalog/feature_selector/operator_recording_install_overlay_readiness_before_build/operator_recording_install_overlay_readiness.txt`.
  - Evidence after build:
    `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_disparity_residual_probe/scenario_residual_catalog/feature_selector/operator_recording_install_overlay_readiness_after_build/operator_recording_install_overlay_readiness.txt`.
  - Metrics: before build installed `AXIS_CONTRACT` was
    `stereo_center_frd:forward_right_down`; after `colcon build
    --packages-select stereo_ground_sim`, installed `AXIS_CONTRACT` is
    `stereodistance_public_frd:forward,right,down,range`.
  - Interpretation: the source recorder schema fix was not enough for live
    recording until the installed ROS overlay was rebuilt. EXP-265 now includes
    this installed overlay gate so future live enablement cannot pass with a
    stale recorder module.

- `EXP-20260618-270-installed-recorder-csv-row-smoke`
  - Result: `PASS`.
  - Evidence:
    `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_disparity_residual_probe/scenario_residual_catalog/feature_selector/installed_operator_recorder_csv_smoke/installed_operator_recorder_csv_smoke_summary.txt`.
  - Metrics: installed module
    `Simulation/install/stereo_ground_sim/lib/python3.10/site-packages/stereo_ground_sim/operator_recorder.py`,
    installed axis contract `stereodistance_public_frd:forward,right,down,range`,
    schema audit valid rows `1/1`, first row `raw_forward_m=1000`,
    `corrected_forward_m=999`, `raw_disparity_px=200`, runtime truth `false`.
  - Interpretation: this is stronger than the installed constant check because
    it exercises the installed callback-to-CSV row path and proves that the
    resulting telemetry can enter the residual preflight schema audit. EXP-265
    now includes this gate before latest live telemetry is considered.

- `guarded_roi_down_range_forward_tiny0p001`
  - Reason: promising independent image-observation branch, but only one long
    ROI seed is complete.
- `bbox_anchor_vertical_global_report_trial_down_guard`
  - Reason: best family-local bbox Down branch, but all-family replay failed
    `15/90` held-out report/trial gates.
