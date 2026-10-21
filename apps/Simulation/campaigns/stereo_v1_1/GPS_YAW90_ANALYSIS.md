# Two yaw sources in the offline campaign

The existing `reprocess` action and `analyze_campaign.py` now compute two
primary-result families from the same raw campaign. No flight is added.
The source captures and older immutable analysis versions are preserved.

| Family | Initial yaw | Main calibration |
| --- | --- | --- |
| `existing_yaw` | Existing G0 Y0..Y6 and G1 nominal-zero initialization | A/B/C, each RPY_IMU and YAW_ONLY |
| `gps_yaw90` | Per-trial noisy camera-RTK baseline perpendicular | A/B/C independently refitted, each RPY_IMU and YAW_ONLY |

M0 uses each family's own initial rotations without fitting. RPY_IMU
updates all camera R/P/Y; YAW_ONLY fixes the same measured camera R/P and
updates yaw. Neither variant estimates camera position. GPS yaw90 is an
initializer, not a new fixed-yaw constraint or yaw-prior penalty. ORACLE
and YAW_IDEAL remain explicitly truth-assisted diagnostics.

## GPS yaw90 input and frame contract

`gps_yaw90.py` reads the existing synthetic camera optical-center RTK
observations, in world ENU `[east,north,up]` metres. With right-minus-left
delta `[dE,dN,dU]`, the NED baseline is `[dN,dE,-dU]`. Baseline compass
azimuth is `atan2(dE,dN)`; candidate forward headings are azimuth minus
90 degrees and plus 90 degrees. Branch scoring, tie handling, Reverse,
consensus and the 5-degree low-margin indication match Workstation's
`workstation_core/baseline_yaw.py` and are tested against that source.

The campaign has synthetic optical IMU R/P, not measured Pixhawk yaw.
It explicitly uses a nominal-north 0-degree reference only to select the
180-degree branch. This is a declared installation assumption, not an
invented Pixhawk sample or truth-based choice. `trial_config.json` records
the measured baseline, both candidates, selected branch, reference source,
warnings and readiness. No drone RTK or GT enters this yaw calculation.
The same selected yaw initializes both cameras; each retains its own
unchanged R/P observation and covariance. Optical R/P is the campaign's
phi/theta parameterization, not raw PX4 body Euler angles. Thus this is a
synthetic campaign counterpart of Workstation's baseline-yaw method,
not a claim that its complete live Pixhawk/body-mount pipeline was run.

R_WC maps optical RDF `[right,down,forward]` vectors to world ENU. Angles
remain radians and covariance radians squared. The existing solver rotation
order is unchanged. A degenerate/non-finite baseline fails closed, retaining
unavailable calibration/M0 results instead of substituting nominal yaw.
Physical independent G1 optical offsets are not magically observed by GPS.

## Files and evaluation

After calibration, path-selection freeze and V0 evaluation, each new
`analysis_versions/analysis_<hash>/` contains:

- `existing_yaw_results.xlsx`: the original-yaw M0/A/B/C results.
- `gps_yaw90_results.xlsx`: baseline-yaw M0/A/B/C results.

Each workbook has Evaluation, Calibration, Inputs and Contract sheets.
The sheets preserve full method/variant IDs, failures, estimated rotations,
attitude errors, position RMSE/P95, detection/validity rates and input hashes.
Missing matched-200 batches stay visible. Oversized sheets split at Excel's
row limit; oversized cells are reported rather than truncated. Failed
corrections and diagnostics are not converted into successful main results.

JSON trial/result paths are already isolated by yaw-case directory:
`.../trial_NNN/GPS_YAW90/<scope>/` versus `.../Y0/<scope>/`, etc.
Original CSV names retain original-yaw rows; separate
`gps_yaw90_calibration_metrics.csv` and `gps_yaw90_evaluation_metrics.csv`
hold the added family. Summary scores are separated by `yaw_source_summaries`;
existing top-level performance summaries remain original-yaw only.
`analysis_latest.json` links both workbooks with repo/output-relative paths;
the summary and final checksums include their hashes.

Both sources share the existing path-selection rule based only on G0 Y0
calibration. Both main calibration families are persisted before the V0
freeze. V0 is not used for refitting or branch/path selection. The existing
25-scenario sensitivity study remains original-yaw only; the two workbooks
contain primary results, not a claim that every sensitivity was duplicated.

XlsxWriter is installed by the existing postprocess setup requirements.
Use the repository's existing `reprocess` workflow; the analyzer command
from the repository root is:

```bash
source /opt/ros/humble/setup.bash
"$STEREO_V1_1_YOLO_PYTHON" apps/Simulation/campaigns/stereo_v1_1/analyze_campaign.py \
  --input-root <existing-QC-valid-campaign> --output-root <new-analysis-output> --mc-trials 100
```

`STEREO_V1_1_YOLO_PYTHON` denotes the configured, prepared postprocess
interpreter; see the existing Simulation setup instructions. The exporter
is part of analysis, not the operator recorder's `session_*.xlsx` path.

Pass criteria are correct branch/frame conversion, shared input pairing,
independent A/B/C fits, truth isolation, disjoint reopened XLSX contents,
and immutable repeat export. There is no new scientific RMSE pass threshold.
