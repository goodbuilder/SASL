# Rejected Or Paused Methods

This file records branches that should not be rerun without a material change
in inputs, observation model, gate, or method.

## Rejected For Current Gate

- Absolute `forward_add` disparity residual after 7x3 multi-burst fusion
  - Reason: EXP-20260618-281 continued the residual sweep after the ray-scale
    PASS and found 16/64 aggregate candidates failed. The representative
    `forward_add log_disparity degree2 ridge1 clip0.5` candidate passed only
    3/4 scenarios; `intrinsic_scale_stress` passed 2/5 trials. It improved
    average range but left residual target worst at `4.591243m` and was less
    stable than ray-scale correction.
  - Interpretation: adding an absolute meter offset to Forward is weaker under
    scale-like camera error because right/down/forward remain on an
    inconsistent ray. Prefer `ray_scale log_disparity` residuals for the next
    StereoDistance export attempt.
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

- `EXP-20260622-368-b15x5-common0p2-inversedepth-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.559493 m`.
  - Interpretation: `inverse_depth` ray-scale reduces aggregate regression
    compared with the product branch, but it does not protect the intrinsic
    scale target rows. Reject as the current guard fallback feature.

- `EXP-20260622-369-b15x5-common0p2-rawforward-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.857175 m`.
  - Interpretation: `raw_forward` ray-scale keeps range RMSE near observation
    (`0.927983 m` to `0.925776 m`) but still misses the intrinsic-scale target
    gate. Reject as a standalone guard fallback feature.

- `EXP-20260622-370-b15x5-common0p2-lograwforward-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.714327 m`.
  - Interpretation: `log_raw_forward` improves the target row versus raw
    forward, but aggregate range RMSE regresses more and the intrinsic-scale
    gate still fails. Reject as a standalone guard fallback feature.

- `EXP-20260622-371-b15x5-common0p2-forwardoverdisparity-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.941890 m`.
  - Interpretation: `forward_over_disparity` keeps aggregate range RMSE near
    observation, but target-row protection is weaker than raw/log-raw-forward.
    Reject as a standalone guard fallback feature.

- `EXP-20260622-372-b15x5-common0p2-disparityoverforward-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.446209 m`.
  - Interpretation: `disparity_over_forward` improves the tracked target row
    versus the other single-feature ratio scouts, but max target and aggregate
    RMSE regress too much. Reject as a standalone guard fallback feature.

- `EXP-20260622-373-b15x5-common0p2-product-deg2-oversample48-local20-scout`
  - Result: `FAIL`, `117/120` passes.
  - Evidence: failures were `intrinsic_scale_stress` seeds `298188`, `298193`,
    and `298198`; seed `298188` residual target max-abs was `2.747281 m`.
  - Interpretation: degree2 product reduces the failure count versus
    single-feature ratio scouts but is worse than degree1 common0.2 on target
    row and aggregate RMSE. Reject this ridge300 quadratic setting.

- `EXP-20260622-374-b15x5-common0p2-product-deg2-ridge3000-oversample48-local20-scout`
  - Result: `FAIL`, `115/120` passes.
  - Evidence: failures were `intrinsic_scale_stress` seeds `298181`, `298188`,
    `298193`, `298198`, and `298199`; seed `298188` residual target max-abs was
    `2.757494 m`.
  - Interpretation: ridge3000 does not stabilize the quadratic product branch;
    it increases the failure count versus ridge300 with similar aggregate
    regression. Reject this regularization setting.

- `EXP-20260622-376-b15x5-common0p2-product-forwardadd-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.670298 m`.
  - Interpretation: product `forward_add` preserves aggregate RMSE almost
    exactly, but it does not apply enough target-row correction. Reject as a
    standalone guard fallback while retaining it as a blending clue.

- `EXP-20260622-377-b15x5-common0p2-logproduct-forwardadd-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.670298 m`.
  - Interpretation: log-product `forward_add` is effectively identical to the
    non-log product additive branch on this scout. Reject as a standalone guard
    fallback.

- `EXP-20260622-387-b15x7-common0p2-inversedepth-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.544095 m`.
  - Interpretation: b15x7 fusion does not fix the inverse-depth target failure
    pattern. Aggregate regression remains small, but strict target protection is
    insufficient. Reject as a standalone guard fallback.

- `EXP-20260623-388-b15x7-common0p2-rawforward-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.836855 m`.
  - Interpretation: raw-forward under b15x7 preserves the aggregate RMSE clue
    (`0.924142 m` to `0.921861 m`) but still misses every intrinsic-scale target
    row. Reject as a standalone guard fallback.

- `EXP-20260623-389-b15x7-common0p2-lograwforward-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.700561 m`.
  - Interpretation: log-raw-forward improves the tracked target row versus
    raw-forward but still fails every intrinsic-scale target row and gives up
    the raw-forward aggregate improvement. Reject as a standalone guard
    fallback.

- `EXP-20260623-390-b15x7-common0p2-forwardoverdisparity-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.914895 m`.
  - Interpretation: forward-over-disparity keeps aggregate RMSE near
    observation, but target-row protection is weaker than the product branch and
    every intrinsic-scale row still fails. Reject as a standalone guard fallback.

- `EXP-20260623-391-b15x7-common0p2-disparityoverforward-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.428112 m`.
  - Interpretation: disparity-over-forward gives the best tracked target row
    among the b15x7 ratio/raw-forward family so far, but still fails the strict
    gate and regresses aggregate RMSE. Reject as a standalone guard fallback.

- `EXP-20260623-392-b15x7-common0p2-disparity-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.544095 m`.
  - Interpretation: plain disparity matches the inverse-depth scout because the
    runtime feature path treats both as the disparity scalar. It still misses all
    intrinsic-scale target rows. Reject as a standalone guard fallback.

- `EXP-20260623-393-b15x7-common0p2-product-forwardadd-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.651248 m`.
  - Interpretation: b15x7 product forward-add preserves aggregate RMSE almost
    exactly, but still under-corrects intrinsic-scale target rows. Reject as a
    standalone additive fallback.

- `EXP-20260623-394-b15x7-common0p2-logproduct-forwardadd-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.651248 m`.
  - Interpretation: log-product forward-add matches the non-log additive branch
    and still under-corrects intrinsic-scale target rows. Reject as a standalone
    additive fallback.

- `EXP-20260623-395-b15x7-common0p2-product-deg2-oversample48-local20-scout`
  - Result: `FAIL`, `118/120` passes.
  - Evidence: failures were `intrinsic_scale_stress` seeds `298188` and
    `298198`; seed `298188` residual target max-abs was `2.725512 m`.
  - Interpretation: b15x7 quadratic product reduces the failure count, but it
    is worse than degree1 product on the tracked target row and aggregate RMSE.
    Reject this quadratic setting.

- `EXP-20260623-396-b15x7-common0p2-product-deg2-ridge3000-oversample48-local20-scout`
  - Result: `FAIL`, `113/120` passes.
  - Evidence: all seven failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `2.735516 m`.
  - Interpretation: stronger ridge slightly lowers the worst residual versus
    ridge `300`, but increases intrinsic-scale failures from two to seven and
    keeps the aggregate RMSE regression. Reject this stronger quadratic ridge
    setting.

- `EXP-20260623-398-b15x7-common0p2-rawforward-forwardadd-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.691247 m`.
  - Interpretation: raw-forward `forward_add` improves aggregate range RMSE
    (`0.924142 m` to `0.917849 m`) and position RMSE (`2.492225 m` to
    `2.489703 m`), but misses every intrinsic-scale target row. Reject as a
    standalone method and retain the aggregate-preserving behavior as a clue.

- `EXP-20260623-399-b15x7-common0p2-rawforward-forwardadd-deg2-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.691247 m`.
  - Interpretation: degree `2` matches the degree `1` failure count and tracked
    target row while slightly worsening aggregate RMSE. Reject this quadratic
    raw-forward additive branch.

- `EXP-20260623-400-b15x7-common0p2-disparityoverforward-forwardadd-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.651248 m`.
  - Interpretation: additive correction does not retain the
    `disparity_over_forward` ray-scale target-row clue and slightly regresses
    aggregate RMSE. Reject as a standalone additive ratio fallback.

- `EXP-20260623-401-b15x7-common0p2-disparityoverforward-ridge3000-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.424058 m`.
  - Interpretation: stronger ridge keeps the tracked target row close to EXP391
    but worsens worst target and aggregate RMSE. Reject this ratio ray-scale
    regularization setting.

- `EXP-20260623-402-b15x7-common0p2-disparityoverforward-ridge30-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.428529 m`.
  - Interpretation: lower ridge behaves close to ridge `3000`; it does not
    reduce intrinsic-scale failures and still worsens aggregate RMSE. Reject
    this ratio ray-scale regularization setting.

- `EXP-20260623-408-b15x7-common0p1-rawforward-forwardadd-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.699468 m`.
  - Interpretation: common0.1 raw-forward `forward_add` improves aggregate
    range RMSE to `0.910094 m` and position RMSE to `2.485815 m`, but it still
    fails every intrinsic-scale target row. Reject as a standalone method and
    keep as an aggregate anchor for blended or gated target protection.

- `EXP-20260623-409-b15x7-common0p1-rawforward-rayscale-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.836726 m`.
  - Interpretation: raw-forward `ray_scale` nearly preserves range RMSE
    (`0.916652 m` to `0.916164 m`) and lowers worst target versus the additive
    raw-forward branch, but it still fails every intrinsic-scale target row and
    worsens position RMSE. Reject as a standalone method.

- `EXP-20260623-410-b15x7-common0p1-rawforward-rayscale-ridge30-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.838256 m`.
  - Interpretation: ridge `30` is effectively tied with ridge `300`; it does
    not reduce intrinsic-scale failures and still worsens position RMSE. Reject
    this lower-ridge raw-forward ray-scale branch.

- `EXP-20260623-411-b15x7-common0p1-rawforward-rayscale-ridge30-clip0p01-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.838256 m`.
  - Interpretation: clip `0.01` exactly matches clip `0.02` at scout precision;
    the clip is not active and failures remain unchanged. Reject this clip
    setting.

- `EXP-20260623-412-b15x7-common0p1-rawforward-rayscale-deg2-ridge30-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.899702 m`.
  - Interpretation: degree `2` lowers the worst target residual but worsens the
    tracked intrinsic-scale row and aggregate RMSE versus degree `1`. Reject
    this quadratic raw-forward ray-scale branch.

- `EXP-20260623-413-b15x7-common0p1-lograwforward-rayscale-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.701293 m`.
  - Interpretation: log scaling improves the tracked seed versus raw-forward
    ray-scale but worsens worst target and range aggregate. Reject this
    log-raw-forward ray-scale branch.

- `EXP-20260623-414-b15x7-common0p1-lograwforward-forwardadd-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.688441 m`.
  - Interpretation: log scaling stays near the raw-forward additive aggregate
    anchor but does not improve it and still fails every intrinsic-scale row.
    Reject this log additive branch.

- `EXP-20260623-415-b15x7-common0p1-forwardoverdisparity-rayscale-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.917725 m`.
  - Interpretation: forward-over-disparity lowers worst target residual versus
    observation, but fails every intrinsic-scale row and worsens aggregate RMSE.
    Reject this ratio ray-scale branch.

- `EXP-20260623-416-b15x7-common0p1-disparityoverforward-rayscale-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.435931 m`.
  - Interpretation: disparity-over-forward improves the tracked seed but
    worsens worst target and aggregate RMSE heavily. Reject this ratio
    ray-scale branch.

- `EXP-20260623-417-b15x7-common0p1-disparity-rayscale-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.549610 m`.
  - Interpretation: plain disparity improves the tracked seed but fails every
    intrinsic-scale row and regresses aggregate RMSE. Reject this plain
    disparity ray-scale branch.

- `EXP-20260623-418-b15x7-common0p1-inversedepth-rayscale-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.549610 m`.
  - Interpretation: inverse-depth matches plain disparity at scout precision and
    is not a distinct improvement axis. Reject this scalar branch.

- `EXP-20260623-419-b15x7-common0p1-disparity-forwardadd-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.659469 m`.
  - Interpretation: plain disparity additive correction slightly improves the
    tracked intrinsic seed but worsens worst target error and aggregate
    range/position RMSE. Reject this scalar additive branch.

- `EXP-20260623-420-b15x7-common0p1-inversedepth-forwardadd-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.659469 m`.
  - Interpretation: inverse-depth additive correction matches the plain
    disparity additive branch at scout precision, so it is not a distinct
    improvement axis. Reject this duplicate scalar additive branch.

- `EXP-20260623-421-b15x7-common0p1-disparityforwardproduct-forwardadd-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.659469 m`.
  - Interpretation: product additive correction regresses aggregate RMSE less
    than the scalar additive branches, but still worsens observation aggregate
    and does not fix intrinsic-scale failures. Reject this additive interaction
    branch.

- `EXP-20260623-422-b15x7-common0p1-logdisparityforwardproduct-forwardadd-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs was `3.659469 m`.
  - Interpretation: log product additive correction is effectively tied with
    unlogged product additive correction and still worsens aggregate RMSE.
    Reject this log interaction branch.

- `EXP-20260623-423-b15x7-common0p1-logdisparity-forwardadd-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs regressed to `3.689243 m`.
  - Interpretation: log-disparity additive correction improves overall target
    max and aggregate range/position RMSE, but still fails all intrinsic-scale
    rows. Reject as standalone deployment candidate, but keep as a promising
    aggregate anchor for further tuning.

- `EXP-20260623-424-b15x7-common0p1-logdisparity-forwardadd-ridge30-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs regressed to `3.689610 m`.
  - Interpretation: lowering ridge from `300` to `30` slightly improves
    aggregate RMSE but does not improve pass count or intrinsic-tail behavior.
    Reject as standalone deployment candidate and continue tuning.

- `EXP-20260623-425-b15x7-common0p1-logdisparity-forwardadd-ridge3-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs regressed to `3.689647 m`.
  - Interpretation: lowering ridge from `30` to `3` is nearly saturated:
    aggregate RMSE improves only slightly while intrinsic-tail behavior slightly
    worsens. Reject as standalone deployment candidate and continue searching.

- `EXP-20260623-426-b15x7-common0p1-logdisparity-forwardadd-ridge0p3-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs regressed to `3.689651 m`.
  - Interpretation: lowering ridge from `3` to `0.3` is saturated and does not
    change pass count or intrinsic-tail behavior. Reject standalone deployment
    and move future search to other tuning axes.

- `EXP-20260623-427-b15x7-common0p1-logdisparity-forwardadd-ridge3-clip0p01-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs regressed to `3.689468 m`.
  - Interpretation: clip `0.01` softens intrinsic-tail regression slightly but
    gives back aggregate gain and does not change pass count. Reject standalone
    deployment and continue searching.

- `EXP-20260623-428-b15x7-common0p05-logdisparity-forwardadd-ridge3-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs regressed to `3.637634 m`.
  - Interpretation: common `0.05` lowers the filtered-population worst target
    and aggregate RMSE, but does not change the intrinsic-scale pass count.
    Reject standalone deployment and keep exploring common-fraction/blend axes.

- `EXP-20260623-429-b15x7-common0p05-logdisparity-forwardadd-ridge3-clip0p01-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs regressed to `3.633976 m`.
  - Interpretation: common0.05 clip `0.01` softens the tracked intrinsic tail
    versus clip `0.02`, but reduces aggregate gain and leaves pass count
    unchanged. Reject standalone deployment.

- `EXP-20260623-430-b15x7-common0p2-logdisparity-forwardadd-ridge3-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs regressed to `3.685612 m`.
  - Interpretation: common `0.2` improves residuals within its own filtered
    population, but the observation baseline is worse than common0.05 and pass
    count remains unchanged. Reject this wider common-fraction direction.

- `EXP-20260623-431-b15x7-common0p05-logdisparity-forwardadd-degree2-ridge3-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs regressed to `3.643976 m`.
  - Interpretation: degree 2 improves aggregate metrics over the degree-1
    common0.05 anchor, but worsens tracked intrinsic-tail behavior and does not
    change pass count. Reject standalone deployment, keep as aggregate anchor.

- `EXP-20260623-432-b15x7-common0p05-logdisparity-forwardadd-degree2-ridge3-clip0p01-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs regressed to `3.633976 m`.
  - Interpretation: degree 2 with clip `0.01` softens the tracked tail but
    gives back most of the degree-2 clip `0.02` aggregate gain. Reject
    standalone deployment.

- `EXP-20260623-433-b15x7-common0p05-logdisparity-forwardadd-degree3-ridge3-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs regressed to `3.643976 m`.
  - Interpretation: degree 3 nearly saturates degree 2; pass count and target
    max are unchanged while aggregate RMSE improves only slightly. Reject
    standalone deployment.

- `EXP-20260623-434-b15x7-common0p05-logdisparity-rayscale-degree2-ridge3-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs regressed to `3.814988 m`.
  - Interpretation: ray-scale correction lowers the worst target max but
    regresses aggregate range/position RMSE and worsens the tracked intrinsic
    target tail. Reject standalone deployment, keep as a blending anchor.

- `EXP-20260623-435-b15x7-common0p05-logdisparity-rayscale-degree2-ridge3-clip0p01-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs regressed to `3.814988 m`.
  - Interpretation: ray-scale clip `0.01` matches clip `0.02` at scout
    precision, so clipping does not resolve aggregate regression or intrinsic
    failures. Reject standalone deployment.

- `EXP-20260623-436-b15x7-common0p05-logdisparity-rayscale-degree2-ridge300-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs regressed to `3.811794 m`.
  - Interpretation: ridge `300` slightly lowers worst target max versus
    ridge `3`, but does not fix aggregate range/position regression or pass
    count. Reject standalone deployment.

- `EXP-20260623-437-b15x7-common0p05-rawforward-rayscale-degree2-ridge3-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs regressed to `3.864106 m`.
  - Interpretation: raw-forward ray-scale correction is weaker than
    log-disparity ray-scale for this branch, with less worst-target improvement
    and more aggregate regression. Reject standalone deployment.

- `EXP-20260623-438-b15x7-common0p05-logdisparityforwardproduct-forwardadd-degree2-ridge3-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs improved to `3.603977 m`.
  - Interpretation: the interaction feature improves selected intrinsic-tail
    rows but worsens overall target max and aggregate RMSE. Reject standalone
    deployment, keep only as a possible blend feature.

- `EXP-20260623-439-b15x7-common0p05-logdisparityforwardproduct-forwardadd-degree2-ridge3-clip0p01-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all `20` failures were `intrinsic_scale_stress`; seed `298188`
    residual target max-abs improved to `3.613977 m`.
  - Interpretation: clip `0.01` reduces aggregate regression versus clip
    `0.02` while retaining part of the tail improvement, but aggregate metrics
    still worsen and pass count is unchanged. Reject standalone deployment.

- `EXP-20260623-440-b15x7-common0p05-logdisparityforwardproduct-rayscale-degree2-ridge3-oversample48-local20-scout`
  - Result: `FAIL`, `119/120` passes.
  - Evidence: the only failure was `intrinsic_scale_stress` seed `298188`, whose
    residual target max-abs improved to `2.668908 m`.
  - Interpretation: this interaction ray-scale branch sharply improves pass
    count but heavily worsens aggregate RMSE and overall residual target max.
    Reject standalone deployment, keep as high-pass-count blend anchor.

- `EXP-20260623-441-b15x7-common0p05-logdisparityforwardproduct-rayscale-degree2-ridge3-clip0p01-oversample48-local20-scout`
  - Result: `FAIL`, `119/120` passes.
  - Evidence: the only failure was `intrinsic_scale_stress` seed `298188`, whose
    residual target max-abs improved to `2.668908 m`.
  - Interpretation: clip `0.01` duplicates clip `0.02`; aggregate regression
    and pass count are unchanged. Reject standalone deployment, keep as
    high-pass-count blend anchor.

- `EXP-20260623-442-b15x7-common0p05-logdisparityforwardproduct-rayscale-degree2-ridge300-oversample48-local20-scout`
  - Result: `FAIL`, `118/120` passes.
  - Evidence: `intrinsic_scale_stress` seeds `298188` and `298198` failed.
  - Interpretation: ridge `300` leaves aggregate RMSE severely regressed and
    loses one pass versus ridge `3`. Reject standalone deployment.

- `EXP-20260623-443-b15x7-common0p05-logdisparityforwardproduct-rayscale-degree1-ridge3-oversample48-local20-scout`
  - Result: `STRICT_PASS`, `120/120` passes.
  - Evidence: no gate failures, but range RMSE regressed from `0.912856 m` to
    `1.105966 m` and target max regressed from `4.131533 m` to `4.861383 m`.
  - Interpretation: strict pass is nonterminal and not sufficient here because
    aggregate accuracy regresses heavily. Reject standalone deployment, keep as
    a high-pass-count blending anchor.

- `EXP-20260623-444-b15x7-common0p1-logdisparityforwardproduct-rayscale-degree1-ridge3-oversample48-local20-scout`
  - Result: `STRICT_PASS`, `120/120` passes.
  - Evidence: no gate failures, but range RMSE regressed from `0.916652 m` to
    `1.108084 m` and target max regressed from `4.348758 m` to `5.094019 m`.
  - Interpretation: common `0.1` keeps strict pass behavior but worsens worst
    residual target error versus common0.05. Reject standalone deployment.

- `EXP-20260623-445-b15x7-common0p02-logdisparityforwardproduct-rayscale-degree1-ridge3-oversample48-local20-scout`
  - Result: `STRICT_PASS`, `120/120` passes.
  - Evidence: no gate failures, but range RMSE regressed from `0.912369 m` to
    `1.106347 m` and target max regressed from `4.255268 m` to `5.000456 m`.
  - Interpretation: common `0.02` keeps strict pass behavior but does not
    reduce aggregate regression and worsens target max versus common0.05.
    Reject standalone deployment.

- `EXP-20260623-446-b15x7-common0p05-logdisparityforwardproduct-rayscale-degree1-ridge0p3-oversample48-local20-scout`
  - Result: `STRICT_PASS`, `120/120` passes.
  - Evidence: no gate failures, but range RMSE regressed from `0.912856 m` to
    `1.105969 m` and target max regressed from `4.131533 m` to `4.861392 m`.
  - Interpretation: ridge `0.3` duplicates ridge `3` at scout precision and
    does not reduce aggregate regression. Reject standalone deployment.

- `EXP-20260623-447-b15x7-common0p05-logdisparityforwardproduct-rayscale-degree1-ridge3-clip0p005-oversample48-local20-scout`
  - Result: `STRICT_PASS`, `120/120` passes.
  - Evidence: no gate failures, but range RMSE regressed from `0.912856 m` to
    `1.105966 m` and target max regressed from `4.131533 m` to `4.861383 m`.
  - Interpretation: clip `0.005` duplicates clip `0.02`; aggregate regression
    is unchanged. Reject standalone deployment.

- `EXP-20260623-448-b15x7-common0p05-logdisparityforwardproduct-rayscale-degree1-ridge3-oversample24-local20-scout`
  - Result: `STRICT_PASS`, `120/120` passes.
  - Evidence: no gate failures, but range RMSE regressed from `0.912811 m` to
    `1.105861 m` and target max regressed from `4.151822 m` to `4.888552 m`.
  - Interpretation: reducing target oversample count from `48` to `24` keeps
    the strict-pass behavior but does not reduce aggregate regression enough.
    Reject standalone deployment.

- `EXP-20260623-449-b15x7-common0p05-logdisparity-rayscale-degree1-ridge3-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all 20 failures are `intrinsic_scale_stress`; aggregate metrics
    are much more stable than the interaction ray-scale branch, with range RMSE
    `0.912856 m` to `0.921242 m` and target max `4.131533 m` to `4.109082 m`.
  - Interpretation: log-disparity-only ray scale is aggregate-stable but does
    not recover intrinsic-scale rows. Reject standalone deployment and use as a
    lower-regression contrast branch.

- `EXP-20260623-450-b15x7-common0p1-logdisparity-rayscale-degree1-ridge3-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all 20 failures are `intrinsic_scale_stress`; common `0.1`
    target max is `4.348758 m` to `4.317681 m`, worse than the common0.05
    log-disparity ray-scale contrast branch.
  - Interpretation: stricter common sampling does not recover intrinsic-scale
    rows and slightly worsens aggregate metrics. Reject standalone deployment.

- `EXP-20260623-451-b15x7-common0p02-logdisparity-rayscale-degree1-ridge3-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all 20 failures are `intrinsic_scale_stress`; common `0.02`
    target max is `4.255268 m` to `4.220372 m`, with range RMSE
    `0.912369 m` to `0.920500 m`.
  - Interpretation: looser common sampling slightly helps RMSE versus common
    `0.05` but does not recover intrinsic-scale rows and worsens target max.
    Reject standalone deployment.

- `EXP-20260623-452-b15x7-common0p05-inversedepth-rayscale-degree1-ridge3-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all 20 failures are `intrinsic_scale_stress`; inverse-depth ray
    scale improves intrinsic seed `298188` from `3.623977 m` to `3.496363 m`
    but regresses range RMSE from `0.912856 m` to `0.947511 m`.
  - Interpretation: inverse-depth ray scale gives selected intrinsic relief
    but is aggregate-worse than log-disparity ray scale and still fails the
    intrinsic gate. Reject standalone deployment.

- `EXP-20260623-453-b15x7-common0p05-rawforward-rayscale-degree1-ridge3-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all 20 failures are `intrinsic_scale_stress`; target max improves
    from `4.131533 m` to `3.985144 m` and range RMSE only moves from
    `0.912856 m` to `0.913125 m`, but intrinsic seed `298188` regresses from
    `3.623977 m` to `3.799632 m`.
  - Interpretation: raw-forward ray scale is aggregate-stable and useful as a
    contrast branch, but it does not solve intrinsic-scale rows. Reject
    standalone deployment.

- `EXP-20260623-454-b15x7-common0p05-rawforward-forwardadd-degree1-ridge3-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all 20 failures are `intrinsic_scale_stress`; range RMSE improves
    from `0.912856 m` to `0.906170 m`, position RMSE improves from
    `2.486601 m` to `2.483988 m`, and target max moves from `4.131533 m` to
    `4.111533 m`.
  - Interpretation: raw-forward forward-add is a low-regression branch but
    still cannot pass intrinsic-scale rows. Reject standalone deployment.

- `EXP-20260623-455-b15x7-common0p05-rawforward-forwardadd-degree2-ridge3-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all 20 failures are `intrinsic_scale_stress`; target max remains
    `4.131533 m` to `4.111533 m`, with range RMSE `0.912856 m` to
    `0.906259 m`.
  - Interpretation: degree 2 duplicates degree 1 at scout precision and does
    not recover intrinsic-scale rows. Reject standalone deployment.

- `EXP-20260623-456-b15x7-common0p1-rawforward-forwardadd-degree1-ridge3-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all 20 failures are `intrinsic_scale_stress`; common `0.1`
    improves range RMSE from `0.916652 m` to `0.910094 m` but target max is
    worse than common `0.05`, moving from `4.348758 m` to `4.328759 m`.
  - Interpretation: stricter common sampling keeps the low-regression shape but
    does not recover intrinsic rows. Reject standalone deployment.

- `EXP-20260623-457-b15x7-common0p02-rawforward-forwardadd-degree1-ridge3-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all 20 failures are `intrinsic_scale_stress`; range RMSE improves
    from `0.912369 m` to `0.905603 m`, but target max is worse than common
    `0.05`, moving from `4.255268 m` to `4.235268 m`.
  - Interpretation: looser common sampling gives the best RMSE in this branch
    but still fails intrinsic rows. Reject standalone deployment.

- `EXP-20260623-458-b15x7-common0p05-rawforward-forwardadd-degree1-ridge3-clip0p005-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all 20 failures are `intrinsic_scale_stress`; intrinsic seed
    `298188` regression shrinks to `3.623977 m` to `3.628976 m`, but range RMSE
    improvement also shrinks to `0.912856 m` to `0.911046 m`.
  - Interpretation: smaller clip softens intrinsic regression but does not
    recover the intrinsic gate and gives up most RMSE improvement. Reject
    standalone deployment.

- `EXP-20260623-459-b15x7-common0p05-rawforward-forwardadd-degree1-ridge3-clip0p01-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all 20 failures are `intrinsic_scale_stress`; range RMSE improves
    from `0.912856 m` to `0.909327 m`, while intrinsic seed `298188` regresses
    from `3.623977 m` to `3.633976 m`.
  - Interpretation: midpoint clip gives midpoint behavior but does not recover
    intrinsic-scale rows. Reject standalone deployment.

- `EXP-20260623-460-b15x7-common0p05-rawforward-forwardadd-degree1-ridge3-oversample24-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all 20 failures are `intrinsic_scale_stress`; range RMSE improves
    from `0.912811 m` to `0.906141 m`, but target max is slightly worse than
    oversample `48`, moving from `4.151822 m` to `4.131823 m`.
  - Interpretation: lower target oversampling keeps RMSE improvement but does
    not recover intrinsic rows. Reject standalone deployment.

- `EXP-20260623-461-b15x7-common0p05-rawforward-forwardadd-degree1-ridge3-oversample12-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all 20 failures are `intrinsic_scale_stress`; range RMSE improves
    from `0.912015 m` to `0.905362 m`, but target max is worse than
    oversample `24`, moving from `4.184992 m` to `4.164993 m`.
  - Interpretation: lower oversampling improves RMSE but continues to worsen
    target max and does not recover intrinsic rows. Reject standalone
    deployment.

- `EXP-20260623-462-b21x5-common0p05-rawforward-forwardadd-degree1-ridge3-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all 20 failures are `intrinsic_scale_stress`; b21x5 improves
    range RMSE from `0.911104 m` to `0.904418 m`, but target max is worse than
    b15x7, moving from `4.219042 m` to `4.199042 m`.
  - Interpretation: more bursts with fewer observations improve RMSE but do
    not recover intrinsic rows and worsen target max. Reject standalone
    deployment.

- `EXP-20260623-463-b7x15-common0p05-rawforward-forwardadd-degree1-ridge3-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all 20 failures are `intrinsic_scale_stress`; target max is
    `4.462819 m` to `4.442819 m`, and range RMSE is `0.921180 m` to
    `0.914699 m`.
  - Interpretation: fewer, larger bursts are worse than b15x7 and b21x5 for
    this branch. Reject standalone deployment.

- `EXP-20260623-464-b21x5-common0p05-rawforward-forwardadd-degree1-ridge3-clip0p01-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all 20 failures are `intrinsic_scale_stress`; clip `0.01`
    softens intrinsic seed `298188` to `3.603210 m` to `3.613209 m`, but range
    RMSE improvement shrinks to `0.911104 m` to `0.907569 m`.
  - Interpretation: softened b21x5 clip does not recover intrinsic rows and
    loses much of the RMSE benefit. Reject standalone deployment.

- `EXP-20260623-465-b15x7-common0p05-lograwforward-forwardadd-degree1-ridge3-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all 20 failures are `intrinsic_scale_stress`; range RMSE improves
    from `0.912856 m` to `0.907625 m`, and intrinsic seed `298188` regresses
    from `3.623977 m` to `3.636818 m`.
  - Interpretation: log raw forward softens the linear raw-forward branch but
    still does not recover intrinsic rows. Reject standalone deployment.

- `EXP-20260623-466-b15x7-common0p05-forwardoverdisparity-forwardadd-degree1-ridge3-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all 20 failures are `intrinsic_scale_stress`; target max is
    `4.131533 m` to `4.111533 m`, and range RMSE is `0.912856 m` to
    `0.906159 m`, matching the raw-forward additive branch.
  - Interpretation: forward-over-disparity is effectively duplicate behavior
    for this scout. Reject standalone deployment.

- `EXP-20260623-467-b15x7-common0p05-disparityoverforward-forwardadd-degree1-ridge3-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all 20 failures are `intrinsic_scale_stress`; intrinsic seed
    `298188` improves from `3.623977 m` to `3.603977 m`, but target max
    regresses from `4.131533 m` to `4.151533 m`.
  - Interpretation: the inverse ratio gives selected intrinsic relief but
    worsens aggregate metrics and still fails the intrinsic gate. Reject
    standalone deployment.

- `EXP-20260623-468-b15x7-common0p05-disparityoverforward-rayscale-degree1-ridge3-oversample48-local20-scout`
  - Result: `FAIL`, `100/120` passes.
  - Evidence: all 20 failures are `intrinsic_scale_stress`; intrinsic seed
    `298188` improves from `3.623977 m` to `3.378348 m`, but target max
    regresses from `4.131533 m` to `4.373617 m` and range RMSE regresses to
    `0.975815 m`.
  - Interpretation: whole-ray scaling increases intrinsic relief but causes
    unacceptable aggregate regression. Reject standalone deployment.
