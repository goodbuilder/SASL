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

## Paused / Under-Evidenced

- `guarded_roi_down_range_forward_tiny0p001`
  - Reason: promising independent image-observation branch, but only one long
    ROI seed is complete.
- `bbox_anchor_vertical_global_report_trial_down_guard`
  - Reason: best family-local bbox Down branch, but all-family replay failed
    `15/90` held-out report/trial gates.
