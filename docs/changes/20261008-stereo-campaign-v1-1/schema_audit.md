# Existing recording-format audit against protocol v1.1

Audited 2026-10-08 from checked-out producer code and a read-only search of this checkout. This is **format capability**, not proof of any recorded run: no `pair_records.jsonl`, `run_summary.json`, `.mcap`, `.db3`, or overlay MP4 was found under this repository. Historical recordings elsewhere were not supplied or qualified.

## Producer and file comparison

The old sweep invokes `run_operator_view.sh --record-full`, so its operator output is periodic `telemetry.csv`, derived compact CSVs and left/right `mp4v` detection-overlay MP4. `operator_recorder.py` writes CSV from a 30 Hz timer using cached latest-topic values. Those rows are display/telemetry samples, **not** one row per original camera image. `--record-debug` adds a rosbag that subscribes to `/left_camera/image` and `/right_camera/image`, but the old sweep does not select Debug mode. The debug bag topic list does not contain the source `/simulation/ground_truth/vehicle_reference_link` PoseStamped or Gazebo `/model/x500_2m/pose`, so raw GT pose at its native rate is not established by that bag alone.

The S2 recorder separately produces `pair_records.jsonl` (authoritative), `pair_records.csv` (derived), `run_metadata.json`, `run_summary.json`, and a validity marker. Its source schema is `s2_atomic_pair_v11` with 427 flat columns. It validates pair keys, row count, queue overflow and hashes for its own three main artifacts. This is a useful **pair-level supplement**, not the protocol's per-source-frame observation/GT package.

Of the protocol's 15 required per-run artifact names, no producer of `flight_reference.json`, `flight_state.jsonl`, `camera_truth.json`, `observations_left.jsonl`, `observations_right.jsonl`, `paired_observations.jsonl`, `ground_truth.jsonl`, `image_index.jsonl`, or `qc_report.json` was found in `apps/Simulation`. Some facts may be derivable from existing logs and pair rows, but file renaming cannot restore omitted raw frames or pose samples. `run_manifest.json`, `effective_config.json`, `events.jsonl`, `runtime.log`, `checksums.sha256` and `raw_images_or_lossless_bag` must also be created/qualified for this campaign, even where existing files provide part of their content.

## Required per-image observation fields

The current pair schema has exact column names for only `run_id`, `attempt_id` and `simulation_epoch` (3/16); several other values exist under pair-level aliases. Exact-name mismatch alone is not a data-loss verdict; the level of observation and provenance decide whether mapping is valid.

| Required field | Existing candidate | Verdict |
| --- | --- | --- |
| `run_id`, `attempt_id`, `simulation_epoch` | Exact pair columns; also `epoch` | Reusable identity for accepted/terminal pairs; not a full frame inventory. Old `attempt_id` is case ID, not retry number. |
| `camera_id` | Separate left/right columns and raw image topic | Derivable when splitting a pair or bag stream; needs explicit side identity per image. |
| `frame_id` | `ImageRecord.sequence_in_side` exists in memory; pair has pair ID | Missing persistent source-frame identity. Pair ID is not frame ID. |
| `image_timestamp_ns` | Pair `left_stamp_ns` / `right_stamp_ns`, from `Image.header.stamp` | Reusable for paired images; unmatched source images need their own index. |
| `receive_timestamp_ns`, `inference_end_timestamp_ns` | No equivalent columns in pair schema | Missing in campaign observation schema; record distinct time domains rather than inventing from pair midpoint. |
| `detection_status` | Pair `left_observation_valid`, `right_observation_valid`, terminal/evaluation status; compact CSV `left_valid/right_valid` | These are pair/telemetry statuses, not a complete explicit per-original-frame detector status. |
| `bbox_xyxy_px`, `bbox_center_uv_px` | Separate left/right bbox coordinates, pixel positions and compact CSV bbox fields | Candidate mapping for paired detections only; original 1280x720 coordinate and letterbox inversion must be independently checked. Missing detection must remain null. |
| `confidence` | Pair `left_confidence` / `right_confidence` | Candidate mapping for paired detections only. |
| `image_width_px`, `image_height_px` | Configured 1280x720, ROS `Image` metadata in optional bag | Must write/verify for every original image; a frozen config value alone cannot prove a particular saved frame's dimensions. |
| `original_image_reference` | Optional debug rosbag raw image topics | No link in pair rows to a bag frame/offset or content hash; requires source frame index. |
| `yolo_weights_sha256` | Session static metadata records model spec/path; no weight hash found in these producers | Missing; resolve actual weight file and hash before capture. |

## Data and coordinate constraints on reuse

- Existing pair `gt_world_enu_x/y/z_m` is world ENU position at a pair alignment, while `operational_x/y/z_m` is a public `baseline_center_frd` value. Their bare `x/y/z` names are **not aliases**; frame metadata must travel with any export.
- The pair schema's camera optical pose coordinates/quaternions are Gazebo diagnostics. They can support `camera_truth.json`, but cannot substitute for noisy RTK optical-center measurements used by estimators.
- Existing GT alignment interpolates world-ENU position only; interpolated quaternion is null. Required shortest-path SLERP, 40 ms bracket flag and raw >=50 Hz GT stream need additions.
- Existing one-to-one image pairer can match nearest frames without a 1 ms cutoff. Its accepted pair rows do not preserve all unmatched source frames.
- S2 validity hashes cover pair JSONL/CSV/metadata. Campaign checksums and QC must cover raw images/bag, source observations, GT, flight state and configuration for each attempt.

## Decision

**Do not use the existing output format unchanged as a v1.1 run package.** Keep S2 pair JSONL and Full/Debug operator artifacts as provenance; add raw-image/GT capture, per-source-frame observations, the explicit schema mapping and campaign QC. An old run can be reused only after its actual files pass all mandatory fields, timing, geometry and integrity checks. No such old run was available in this checkout to test.
