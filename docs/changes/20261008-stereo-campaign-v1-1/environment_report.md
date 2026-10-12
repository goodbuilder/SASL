# 39-run stereo campaign v1.1: environment inventory

Recorded 2026-10-08 KST. This is a source and installed-tool inspection, not a Gazebo flight result. Campaign completion: **0/39 verified in this review**. Existing data have not yet been qualified against the new raw-data schema.

## Authority and workspace

- Protocol: `작업지시서/02_Codex_작업지시문.txt`, `03_experiment_spec.json`, `04_run_manifest.json` (39 slots: G0 30, G1 9). The original package is untracked. The numerical spec, manifest and reference generator now have hash-pinned copies under `apps/Simulation/campaigns/stereo_v1_1/protocol/` for a future notebook clone. The manifest/generator are byte-identical; the spec changes only its nonfunctional historical checkout-path hint to a neutral warning, as recorded in `provenance.json`. These new files must be included in Git before handoff.
- Repository HEAD: `42b16e0305120c769aaa6483254d19c08fbf55bf`, branch `work/20260906`, last commit dated 2026-10-05 15:14 KST. `git status --porcelain=v1` reported 108 existing changes/untracked entries before this report. In particular, `apps/Simulation/config/simulation.env` is modified. No existing change was reset or overwritten.
- The old commit `6e428ca...` and dated checkout path in the instruction are historical search hints, not the checked-out state.

## Installed and discoverable software

| Item | Direct observation | Campaign implication |
| --- | --- | --- |
| Gazebo | `gz sim --versions` reports `8.11.0`; executable `/usr/bin/gz` | Installed; no scene was launched in this review. |
| ROS 2 | `/opt/ros/humble/bin/ros2`; `ros-humble-ros-base` and `ros-humble-desktop` packages installed | Humble is installed. `ros2 --version` is not a supported CLI option, so it is not version evidence. |
| PX4 | No repo-local `apps/Simulation/PX4-Autopilot` checkout or home-top-level PX4 checkout. Two other machine-local checkouts exist: `~/바탕화면/DroneEmergency/PX4-Autopilot` (`v1.18.0-beta1-4-gd448e3f5b0-dirty`) and `~/Capstone/source/PX4-Autopilot` (`v1.17.0-alpha1-1214-g7b1e54889d-dirty`), both with SITL binaries. | Neither is demonstrated to be the required v1.17.0 source/patch. The runner's default search does not include these exact paths. Resolve version, source changes, and reproducible setup before flight. Do not silently use either binary. |
| Workspace | `apps/Simulation/install/` exists; `apps/Simulation/.venv/` does not | Installed ROS artifacts exist, but source/build consistency and runtime import remain unverified. |
| Disk | `df -h .`: 605 GiB available | A short raw-image capture must establish actual storage rate before 39 runs. |

## Existing simulation chain

- `apps/Simulation/run_operator_view.sh` starts the operator/SITL chain and accepts `--px4-dir`; `stereo_ground_sim/launch/stereo_ground_sim.launch.py` bridges `/left_camera/image` and `/right_camera/image` as ROS `sensor_msgs/Image`.
- `run_auto_attitude_sweep.sh` restarts Gazebo/PX4 per case and has `--plan-only`, but its current contract uses center/zigzag waypoint paths, right-camera optical-RDF perturbations, and five-second holds at each waypoint. It is **not** the specified continuous P0–P4/V0 39-run campaign. Do not run the old sweep as substitute data.
- `scripts/px4_auto_mission.py` sends `MAV_FRAME_LOCAL_NED` position/velocity setpoints and reads `LOCAL_POSITION_NED`. Its waypoint CSV aliases `x` to north and `y` to east. These are local-NED aliases, **not** the protocol's world-ENU `x=east, y=north`; an observed spawn/home transform is required before connecting the reference path.
- `stereo_ground_sim/stereo_ground_sim/gazebo_ground_truth_adapter.py` reads Gazebo Pose_V source stamps and outputs `/simulation/ground_truth/vehicle_reference_link`; the source identifies quaternion storage as `xyzw`. Source code alone does not establish achieved GT rate, antenna lever arm, or interpolation quality.
- The S2 research recorder already writes atomic pair JSONL, a derived CSV, run metadata/summary and hashes, and marks queue overflow/invalid finalization. It is enabled explicitly. This is useful infrastructure but has not been mapped against every required v1.1 field.
- The operator recorder has an optional debug rosbag topic list containing both **raw** image topics and camera_info. Its normal MP4 output is detection overlay and lossy. Verify rosbag start/stop, complete frame stamps, lossless serialization, dropped-frame counts, and storage before relying on it.
- `simulation.env` currently names `yolov8n_uav.pt` with confidence `0.10`; no model file was found under `apps/Simulation` at depth four. The actual resolved weight file, hash, NMS threshold, and letterbox mapping are not frozen yet.

## Coordinate contract review before modification

| Quantity | Verified source contract | Open verification |
| --- | --- | --- |
| Protocol world position | Landing-pad-centered ENU: `x=east/right`, `y=north/forward`, `z=up`, metres; camera centers `(-12.5,0,0.1)` and `(12.5,0,0.1)` m | Gazebo optical centers and observed transforms under the generated world. |
| Camera orientation/image | OpenCV optical RDF: `x=right`, `y=down`, `z=forward`; `R_WC` maps optical to world; nominal 20° elevation, new north-to-east heading; image `u,v` pixels | Current `camera_optical_rdf` perturbation uses a different convention; verify generated rotation matrices/quaternion order and unit-vector projections. |
| PX4 motion | Local NED `north,east,down` and body FRD `forward,right,down`; current mission code uses local NED | World-ENU to local-NED origin, orientation, altitude reference and heading readback for each fresh PX4 instance. |
| Existing public measurement | `baseline_center_frd` order `forward,right,down` (metres); existing raw camera rays use optical RDF | Do not copy center-frame fields into left/right optical fields or treat public FRD as world ENU. |
| GT and sensor observations | Gazebo `vehicle_reference_link` is world pose; camera RTK optical centers and camera IMU are separate synthetic/physical observations; drone RTK is another stream | `base_link`/reference/antenna transforms, covariance units, GT rate, source-image timestamps, and 40 ms interpolation gate. |

The current locally modified `simulation.env` has `CAM_L/R_OPTICAL_PITCH_DEG=8.0` and camera PX4 body pitch `8.0`; HEAD had `20.0`. Protocol v1.1 defines `R_WC_nominal` for 20° elevation. The world SDF template also contains a 5 m camera model pose while configuration says optical height 0.1 m; the generated SDF path and Gazebo transform must be checked before labeling either as the rendered optical center. This is an unresolved configuration conflict, not a reason to overwrite the user's setting.

## First-stage assessment and next gate

The source inventory is complete enough to locate reuse points and reject the old sweep as a direct runner. No run was started and no new measurement was claimed. Before campaign execution: (1) identify a reproducible, compatible PX4 source and actual runtime; (2) select the now-added G0/G1 20° campaign profiles and verify actual Gazebo camera transforms; (3) inspect YOLO resolution/thresholds and source image timestamps; (4) map required raw fields to topics and test lossless recording; (5) establish ENU-to-local-NED transform, seed consumers, and GT/mesh/antenna contracts. Preserve any earlier data and all current worktree modifications.

## Reference/package check (2026-10-08)

`python3 작업지시서/05_generate_reference.py --output-dir /tmp/sasl-campaign-ref.c5CETG` returned `REFERENCE_GEOMETRY_PASS`, 39 planned slots and six paths. An independent read of the generated rows confirmed each ENU start/end point, `z=y*tan(20°)`, lateral amplitudes/signs, zero endpoint speeds, and sample spacing no greater than 20 ms. These are geometric checks only.

The packaged reference JSONs are not byte-identical to local regeneration: line endings differ (packaged CRLF, local LF), and floating-point values differ by at most `5.684341886080802e-14`. Their structure and practical geometry agree, but an exact regenerated-file hash comparison cannot be used as a pass criterion across this environment. All 15 present package files match the hashes listed in `SHA256SUMS.txt`; one listed non-execution handoff file, `00_학생에게_보낼메시지.txt`, is absent. `sha256sum -c` also fails directly on the CRLF-formatted checksum list because carriage returns enter its filenames; a read-only Python hash check confirmed the 15 present files. The missing file and line-ending issue are package defects to report, not permission to invent its contents or change numeric protocol values.

## 2026-10-09 execution qualification addendum

The historical inventory above was written before the campaign. In the actual checkout, Gazebo 8.11.0, PX4 SITL v1.17.0, ROS 2 Humble, the pinned UAV YOLO weight, isolated Ultralytics 8.4.127 and CUDA inference were exercised. G0/G1 optical poses matched Gazebo transform readback. Each flight observed its PX4-local-NED origin from world-ENU GT/local telemetry before commands, rather than using configured spawn coordinates. Original left/right image bags ran at 20 Hz; GT was observed at 50 Hz with image-time position interpolation and XYZW shortest-path SLERP. The campaign completed 39/39 source-QC-valid slots, including 7 recorded V0 science-entry performance failures; no raw recording technical failure occurred. The 39-attempt checksum/QC readback and no-op resume both passed. The exact repetition sequence audit found no pair with completely identical image-pixel and GT pose sequences, but independent stochastic seed effects were not established. The physical GNSS antenna lever arm remains unknown; simulated navsat and reference-link geometry are documented in `vehicle_geometry.json`.
