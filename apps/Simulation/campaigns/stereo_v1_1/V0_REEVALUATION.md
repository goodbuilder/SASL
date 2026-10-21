# V0-only reevaluation

Use this workflow when a new V0 recording should be evaluated against a frozen
calibration analysis. It copies the matching calibration result bundles and
camera sensor draws plus checksum-verified trial configurations into a new
folder, then evaluates the new V0 observations. It does not rerun camera
calibration MC100 or the sensitivity calibration.

## Prepare the folder

The evaluator checks that Python dependencies match the parent analysis freeze.
From the repository root, create its small analysis environment and source ROS:

```bash
SASL_ROOT="$(git rev-parse --show-toplevel)"
python3 -m venv --system-site-packages "$SASL_ROOT/apps/Simulation/.venv-analysis"
"$SASL_ROOT/apps/Simulation/.venv-analysis/bin/python" -m pip install \
  --requirement "$SASL_ROOT/apps/Simulation/campaigns/stereo_v1_1/requirements-analysis-freeze.txt"
source /opt/ros/humble/setup.bash
export PYTHONPATH="$SASL_ROOT/apps/Simulation${PYTHONPATH:+:$PYTHONPATH}"
```

Prepare a separate output directory from the original frozen analysis and a
development V0 attempt. `prepare` prints the new folder path in
`output_relative_to_campaign`.

```bash
"$SASL_ROOT/apps/Simulation/.venv-analysis/bin/python" -m campaigns.stereo_v1_1.reevaluate_v0 prepare \
  --campaign-root "$SASL_ROOT/apps/Simulation/recordings/stereo_v1_1/paper1_v1_2_001" \
  --parent-analysis-root "$SASL_ROOT/apps/Simulation/recordings/stereo_v1_1/paper1_v1_2_001/analysis_versions/analysis_9cfd7c6cd10b" \
  --pilot-attempt-dir "$SASL_ROOT/apps/Simulation/recordings/stereo_v1_1/paper1_v1_2_001/개발시험/spawn_N600_V0_vertical_20261020_01/01_V0_N600"
```

The folder contains the matching group/repetition's frozen calibration result
bundles and shared sensor draws, the per-case `trial_config.json` files needed
by the sensor-only and GPS-yaw evaluation branches, the parent freeze/context,
and a verified copy of the V0 source recording. Trial configurations are
checked against the parent case checksum manifest and indexed separately from
the calibration result bundles. The folder also indexes the copied lossless
image/state bags. The N=600 pilot maps to `G0-V0-R02`, so it reuses 8,000
calibration batches (five P routes × 100 trials × eight yaw cases × two scopes).

## Run the pinned detector

Use the path printed by `prepare` as `REEVAL` below. The isolated YOLO runtime
uses the pinned Ultralytics 8.4.127 model and the pilot workbook's detector
settings. This machine has no CUDA device available, so the current run uses
CPU inference.

```bash
REEVAL="$SASL_ROOT/apps/Simulation/recordings/stereo_v1_1/paper1_v1_2_001/analysis_versions/v0_reevaluations/<folder printed by prepare>"
"$SASL_ROOT/apps/Simulation/.venv/bin/python" -m campaigns.stereo_v1_1.postprocess_detections \
  --attempt-dir "$REEVAL/v0_source/attempt" \
  --output-dir "$REEVAL/v0_source/attempt/yolo-8.4.127" \
  --model "$SASL_ROOT/apps/Simulation/.local/campaign_assets/sha256/8a2204bf2ed7c09b9dbabeaa1c1e984ab482c10325e2387150e23d8daa3cb777/yolov8n_uav.pt" \
  --device cpu --confidence 0.1 --nms-iou 0.7 --imgsz 736 --target-hz 20 \
  --parameter-source development_pilot_workbook_parameters
```

## Evaluate V0 with the frozen calibration

Return to the ROS environment and use the dependency-matched analysis Python:

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="$SASL_ROOT/apps/Simulation${PYTHONPATH:+:$PYTHONPATH}"
"$SASL_ROOT/apps/Simulation/.venv-analysis/bin/python" -m campaigns.stereo_v1_1.reevaluate_v0 evaluate \
  --reevaluation-root "$REEVAL"
```

The output includes `reevaluation_summary.json`, `v0_evaluation_metrics.csv`,
`calibration_run_index.json`, per-case evaluation/checksum files, and
`reevaluation_status.json`. `calibration_recomputed` and
`sensitivity_calibration_recomputed` remain false. V0 position estimates are
computed for the existing 100 MC trial indices using the copied calibration
results and exactly matched camera sensor draws.

The available N=600 recording is marked as a development pilot, not a 39-slot
campaign result. Its science reference matches the source V0, while its spawn
changes from `[E,N,U]=[0,415,6] m` to `[0,600,1.05] m` and it uses a vertical
liftoff gate. Interpret the resulting metrics as this combined development
condition, not as an isolated north-coordinate change or an independent final
test.

Coordinate contract: the V0 ground truth and synthetic RTK positions are world
ENU `[east,north,up]` in metres; PX4 local position is NED
`[north,east,down]`; stereo rays use optical RDF `[right,down,forward]`; image
coordinates are pixel `[u,v]`. The preparation and evaluator verify the camera
geometry and the shared MC sensor draws before calculating position metrics.
