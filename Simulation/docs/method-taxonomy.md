# Method Taxonomy

This taxonomy keeps correction-method names stable across experiments.

## Method Families

| Family | Meaning |
| --- | --- |
| `raw_stereo` | Uncorrected stereo estimate. |
| `range_only` | Corrects Forward/range while leaving lateral axes raw. |
| `origin_bbox_reprojection` | Uses Origin-BBOX or bbox coordinates to build reprojection/profile corrections. |
| `bbox_anchor` | Uses bbox anchor/cell/range/disparity profile families. |
| `bbox_geometry` | Uses non-center anchors, bbox edges, width, height, area, or shape features. |
| `roi_silhouette` | Uses ROI image observation such as silhouette centroid/top/bottom. |
| `extrinsic_refinement` | Fits or applies camera extrinsic correction or global bias. |
| `tracking_filter` | Uses temporal filtering or Kalman-style smoothing. |
| `multi_source_fusion` | Selects or fuses multiple source estimates per axis or range bin. |
| `guard_policy` | Adds runtime-safe no-op/range/cell/sample guards to another method. |

## Common Baselines

| Baseline | Current Role |
| --- | --- |
| `raw_stereo` | Required comparison for all experiments. |
| `range_only_forward_fusion` | Stable Forward baseline; Right/Down no-op. |
| `bbox_anchor_axis_guard_rg0p1_dg0_fg1` | Current best all-family Right/Forward non-noop candidate; Down no-op. |
| `bbox_anchor_vertical_global_report_trial_down_guard` | Best family-local bbox-derived Down branch; not all-family. |
| `guarded_roi_down_range_forward_tiny0p001` | Best under-evidenced image-observation branch. |

## Claim Types

Use precise claim types instead of broad labels:

```text
forward_only_improvement
right_forward_improvement
down_improvement
all_axis_nonregression
all_axis_improvement
family_local_improvement
runtime_safe_guard
coverage_fix
diagnostic_only
```

## Axis Policy Labels

Use these labels in README and verdict files:

| Label | Meaning |
| --- | --- |
| `corrected` | Axis is intentionally changed by the method. |
| `protected_noop` | Axis intentionally remains raw unless a guard allows correction. |
| `range_guarded` | Correction is enabled only in raw range/Forward bands. |
| `cell_guarded` | Correction is enabled only for trained feature cells. |
| `sample_guarded` | Correction is enabled only when per-sample conditions pass. |
| `fallback_raw` | Runtime fallback keeps raw output. |

## Reference Expectations

Examples:

- `extrinsic_refinement` should have at least `L2/R2` rationale because it is
  grounded in camera geometry.
- `roi_silhouette` can start at `L1/R1` if justified by previous ROI results,
  but should become `L2` when tied to an observation model.
- `bbox_geometry` or `KNN residual` without a physical observation model should
  be marked `L0` or `L1`, not promoted directly.
- Any method proposed from papers should record the source in
  `references.md` and use `L3/R3` only when the source directly supports the
  method.

