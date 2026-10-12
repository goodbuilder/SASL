# v1.1 PX4, optical pose, and model preflight

Observed 2026-10-09 KST. Method: **physical camera/PX4 boundary preflight**. Purpose: qualify the exact SITL source and compare actual Gazebo camera poses with the protocol before any of the 39 science flights. Inputs are the frozen v1.1 spec/manifest, G0/G1 profiles, `x500_2m` SDF, and repo-owned PX4 preparation helper. Output frames: camera centers and `R_WC` in world ENU metres/optical RDF; PX4 local position in NED metres. PASS for optical pose requires center error <1 µm and rotation error <0.00001° relative to `Rz_world(-psi) @ R_WC_nominal`. A compatible PX4 build requires exact tag, all submodules initialized, interface equality and a passing build fingerprint. These checks do not qualify trajectory control or raw capture.

## PX4 source and build

- An independent ignored checkout was cloned from an existing local Git mirror at exact tag `v1.17.0`, commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`; its `origin` was set to the official PX4 Git URL for future fetches. The mirror's dirty working tree was not copied. All 39 recursive submodules are initialized at the tagged commits.
- Repo helper `scripts/prepare_px4_gz_magnetometer_frame.py prepare` applied its known stock-v1.17 patch to `GZBridge.cpp`: Gazebo sensor-local FLU -> PX4 body FRD. Patched source SHA-256: `ecf7581cc32ef9f6f76dfee78e6507c821bddbe94d116332c0c3fbee7b75d4c9`.
- `make px4_sitl_default -j4` completed. The first sandboxed build was stopped by unwritable `/run/user/1000/ccache-tmp`; the retried build completed. `record-build` and `check-build` both verified binary SHA-256 `e75ff660531bbfd26e08a9e2857bed7daf4ae40cf7af2e266313fb12cf3bcfce`, camera airframe SHA-256 `38220533f928543b6df509e717566b10a5dbef600e432daea79e6c96d8b37e82`, patched bridge and PX4 HEAD. PX4 `msg/SensorGps.msg` equals repo-local canonical `workspace/main/src/px4_msgs/msg/SensorGps.msg`; that ROS package is at `86d8239e962f6939e05c3737784f60c02fa884db` with a matching installed overlay stamp.
- A G0 headless operator smoke (`--no-yolo`, `observation_mode:=ideal_reference_point`, `x500_2m`, reuse checked builds) started three PX4 instances on target, left-camera and right-camera namespaces. After sourcing the repo ROS overlay, one actual `vehicle_attitude` message was read from each namespace. The smoke was stopped after readback; no flight command was sent and no process remains. This is **SITL transport readiness**, not science data.

## Gazebo physical optical readback

For each group, `stereo_ground_sim.launch.py` generated an `x500_2m` world using the campaign profile. The resulting world SDF SHA-256 was `69c901a19bc99d0db60140f24577df70060ccfdf89708092f17aa0d0251b9525` (G0) and `8bfcf61d1982a91c28d77c7340099544bcb925d3f025b99331ebf0ea992af0df` (G1). Gazebo 8.11.0 ran each world headlessly. We read its `pose_static` topics for both camera models, composed the model and `optical_link` **XYZW** quaternions, and converted link `(+X,+Y,+Z)` to optical `(Forward,-Right,-Down)`. Expected `R_WC` was calculated independently from the protocol JSON.

| Group/eye | Observed optical center, world ENU m | Observed heading from +Y toward +X | Observed elevation | Rotation error |
| --- | --- | ---: | ---: | ---: |
| G0 left | `(-12.5, 0, 0.1)` | `-0.000000012°` | `19.999999977°` | `0.000000026°` |
| G0 right | `(12.5, 0, 0.1)` | `-0.000000012°` | `19.999999977°` | `0.000000026°` |
| G1 left | `(-12.5, 0, 0.1)` | `+0.499999988°` | `19.999999980°` | `0.000000031°` |
| G1 right | `(12.5, 0, 0.1)` | `-0.500000012°` | `19.999999980°` | `0.000000026°` |

Maximum center residual was `1.78e-15 m`; maximum matrix-entry residual was `3.75e-10`. All four pass the stated optical criterion. Raw Gazebo messages: [G0 left](evidence/G0-left-pose.jsonl), [G0 right](evidence/G0-right-pose.jsonl), [G1 left](evidence/G1-left-pose.jsonl), [G1 right](evidence/G1-right-pose.jsonl). PX4 camera body pitch remained the separately configured 8° during integrated G0 smoke; it was not substituted for optical truth.

## YOLO asset and settings

The two discovered historical `yolov8n_uav.pt` copies have the same SHA-256 `8a2204bf2ed7c09b9dbabeaa1c1e984ab482c10325e2387150e23d8daa3cb777` and size 6,269,475 bytes. One was copied and hash-verified into ignored Simulation-owned, content-addressed storage; the bare runtime alias points to it. The repo-local `model_manifest.json` pins the digest and relative paths, and `import_yolo_weights.py --source <supplied-file>` recreates the owned copy on another checkout. `--check` passed here. The weight loads with class `{0: UAV}` under the currently available Ultralytics 8.4.82.

The active Simulation config requests confidence `0.10`, `imgsz=736`, PyTorch backend and isolated Ultralytics `8.4.127`. Current inference code does **not** pass an explicit NMS IoU argument. The available 8.4.82 package has a default of `0.7`, but that is not proof of the effective 8.4.127 campaign runtime. The integrated smoke deliberately disabled YOLO. Detector runtime, NMS and performance are not yet qualified; do not count this as a YOLO capture trial.

## World ENU versus PX4 local NED origin

The existing waypoint generator explicitly maps world ENU `(east,north,up)` to PX4 local `(north,east,down)` by subtracting a configured origin. This must use an **observed** per-run origin. In a disarmed, stationary G0 integrated smoke, Gazebo `vehicle_reference_link` stayed at approximately `(0,415,1.02645)` world ENU m while PX4 `vehicle_local_position_v1` was near zero. Three sequential (not exactly time-paired) reads imply origin world ENU estimates of `(0.0396,415.0018,0.9731)`, `(0.0274,415.0106,0.9854)`, and `(0.0085,414.9756,0.9957)` m. Raw readbacks are [local 1](evidence/local-1.yaml), [local 2](evidence/local-2.yaml), [local 3](evidence/local-3.yaml), [GT 1](evidence/gt-1.yaml), [GT 2](evidence/gt-2.yaml), [GT 3](evidence/gt-3.yaml).

**This is a finding, not a frozen transform.** The configured target spawn altitude is 6 m, but the disarmed model settled near 1.03 m and PX4 local Z was near zero after estimator resets. Thus using `(0,415,6)` as the PX4 local origin for the science path would be wrong in this observed state. The reads were sequential and the PX4 value is an estimate with noise/reset history; a synchronized transform at the qualified science start and a small directional motion test remain required before commanding the reference trajectory. World ENU `x/y/z`, PX4 local NED `x/y/z`, and camera optical RDF must never be exchanged by bare axis names.

## Gate result

PX4 source/build and G0/G1 physical optical pose: **PASS**. Pinned model file import/class: **PASS**. Formal YOLO runtime, synchronized world-to-local transform under flight, source image capture and flight control: **NOT_RUN**. Campaign completion remains **0/39**. The PX4 checkout/binary and model bytes are ignored build/machine-local products; reproducible source/configuration and setup instructions are under `apps/Simulation`, and all newly required source files must be added to Git before notebook handoff.
