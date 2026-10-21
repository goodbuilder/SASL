# x500_2m reference height midpoint

User request: move the reference to the midpoint of the visual height.
Source baseline: HEAD `6810b22` plus the existing dirty working tree; unrelated
campaign and Workstation changes were preserved. No historical recordings edited.

## Contract and implementation

- Geometry owner: `models/x500_2m/model.sdf` and its checked-in DAE meshes.
  Visual bounds in base_link FLU are Z = -1.0265138481 to +0.3083 m.
- Reference: `[0,0,-0.35910692405]` m in base_link FLU, identity RPY;
  `project_defined_vehicle_reference_at_visual_height_midpoint_v1`.
  X/Y stay at the original origin. This is body-attached, not a dynamic world
  bounding-box center, a physical antenna location, or a mass center.
- Coordinate owner: model-specific helpers in `coordinate_frames.py`.
  `x500_15m` retains its zero-offset reference and legacy constants.
- Adapter boundary: compose model/link pose first, then apply
  `p_world_reference = p_world_base + R_world_base * offset_base` once.
  Output is world ENU `[east,north,up]` metres with unchanged simulation stamp
  and XYZW orientation. PX4 FRD/NED attitude is not used for this conversion.
- Visualization: adapter keeps the original base pose for its mesh; the
  reference-consuming RViz path subtracts the rotated offset before placement.
- Evaluation/recording: model-specific definition is propagated to research
  metadata and campaign GT JSONL, with parent FLU offset recorded in new JSONL.
  Reference positions are shifted before time interpolation. Baseline-center
  FRD and optical RDF downstream contracts are unchanged.
- KEEP: model-specific helpers and adapter conversion; a global offset change
  would incorrectly alter x500_15m. No new dependencies or runtime GT input
  were added to measurement methods. Physical navsat/IMU/flight origins stay put.

## Verification

1. Initial focused run: 25 PASS, 1 FAIL. The existing recorder signal test
   could not import `px4_msgs`; this was an unsourced ROS overlay, not a changed
   coordinate result. Re-run after sourcing the repository ROS overlay passed.
2. Source tests: PASS, 62 tests across vehicle reference, evaluation, recorder,
   SDF protection, GT-only behavior, terminal records/accounting, late GT,
   RViz and GT provider. Then one additional consumer mesh-origin regression
   was added; the vehicle-reference file passed all 18 tests (63 distinct
   passing tests across these runs).
3. Real ROS smoke: PASS inside the reference test. The real adapter subprocess
   reader/thread and ROS publishers consumed synthetic Pose_V with 90-degree
   pitch. Reference ENU was `(9.64089307595,20,30)` while the mesh stayed at
   `(10,20,30)`; simulation stamp `(12,345)` was preserved. No flight or physical
   hardware was exercised. Missing-model and unknown-contract negative tests pass.
4. Installed source parity: PASS for all seven changed package Python modules
   and the model SDF after colcon install. SDF SHA-256:
   `f7b52fbffb47f144800683ee88ba34e6a5ac59412343ff39542c6882cb4c38da`.
5. `python3 tools/check_portable_paths.py`: PASS. `git diff --check`: PASS.

Reproduce source tests from the repository root:

```bash
source /opt/ros/humble/setup.bash
source workspace/main/install/setup.bash
env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest -q \
  apps/Simulation/tests/test_vehicle_reference_ground_truth_contract.py \
  apps/Simulation/tests/test_evaluation_pipeline.py \
  apps/Simulation/tests/test_stereo_v1_1_state_recorder.py \
  apps/Simulation/tests/test_sdf_contract.py \
  apps/Simulation/tests/test_s1_ground_truth_only_contract.py \
  apps/Simulation/tests/test_s2_research_terminal_paths.py \
  apps/Simulation/tests/test_rviz_ray_visualization.py \
  apps/Simulation/tests/test_ground_truth_provider.py \
  apps/Simulation/tests/test_s2_late_gt_evaluation.py \
  apps/Simulation/tests/test_s2_terminal_accounting.py \
  apps/Simulation/tests/test_research_terminal_coordinator.py
```

Install from `apps/Simulation` after sourcing ROS:

```bash
colcon --log-base /tmp/sasl-midpoint-colcon-log build \
  --base-paths stereo_ground_sim --build-base /tmp/sasl-midpoint-colcon-build \
  --packages-select stereo_ground_sim --event-handlers console_direct+
```

The `/tmp` paths are disposable build/log caches. Source and installed output
are repository-relative. Restart any running Simulation to load the new model
and node code. End-to-end Gazebo/PX4 flight and GUI visual inspection: NOT_RUN;
the fresh runtime evidence is the real ROS smoke with synthetic Gazebo input.
Old recordings retain the origin contract; relabeling them as midpoint data
would be incorrect. Existing frozen coordinate-source hashes must not be
silently updated to certify old evidence against new code.

Architecture refresh: NOT_REQUESTED, risk LOW, affected nodes
`coordinate_frames`, `gazebo_ground_truth_adapter`, `evaluation_pipeline`,
`rviz_publisher`; reference-coordinate contract changed, ownership did not.
Suggested command: use `$archify` to refresh the Simulation architecture reader.

## Follow-up: apply the same method to x500_15m

The user subsequently requested the midpoint reference for x500_15m as well.
This supersedes the earlier statement that x500_15m retains the origin; the
earlier test observations describe the first change, not the final contract.

- Fresh SDF/DAE calculation: base visual scale 24.5245, bottom
  -6.19487643775 m; rotor hub top `1.4715 + 0.04 + 0.1784/2 = 1.6007` m.
  The body/bell mesh files are byte-identical to x500_2m but SDF scales and
  visual offsets differ, so the midpoint was recalculated rather than scaled
  from the smaller model's midpoint.
- Height is 7.79557643775 m. Reference base_link FLU translation is
  `[0,0,-2.297088218875]` m, identity RPY; top/bottom distances are both
  3.897788218875 m. Model geometry, physics and navsat origins are unchanged.
- Default x500_15m constants now match the model-specific helper and SDF.
  The existing shared adapter/evaluation/visualization paths apply the new
  value. x500_2m retains its own -0.35910692405 m translation.
- Current qualification source validation uses the current offset instead of
  a literal zero pose. Historical sealed schema and anchor remain untouched;
  a new targeted test proves historical origin provenance cannot certify the
  midpoint runtime, even independently of source-hash rejection.
- PASS: the source regression command above, 67 tests, including geometry
  checks and real ROS adapter smoke for both models. x500_15m at 90-degree
  pitch publishes reference `(7.702911781125,20,30)` ENU from base
  `(10,20,30)`, while keeping the mesh at the base and preserving the stamp.
- SKIPPED: 97 tests in `test_s2_correction_qualification_anchor.py` and
  `test_s2_correction_qualification_anchor_hardening.py`; the sealed fixture
  `tests/fixtures/qualification/exact-profile-freeze-88a360f` is unavailable.
  This is not a claim of sealed-evidence requalification.
- PASS: portable path policy and whitespace checks. Source x500_15m SDF SHA:
  `25c60a91df681fe84ee825d4abf5f15c08d4db4b57e55b8e052ee63b64aa5b45`.
- PASS: colcon build/install (one package, 30.4 s); installed import reports
  both expected offsets, both installed SDFs and all eight affected Python
  modules match source bytes.
- Runtime scope remains synthetic Gazebo input through real ROS; no Gazebo/PX4
  flight or GUI visual verification was performed. Restart a running Simulation
  to load new code/assets; old recordings keep their historical reference.

Installation uses the same colcon command above with disposable build/log
directories `/tmp/sasl-midpoint-15m-colcon-build` and
`/tmp/sasl-midpoint-15m-colcon-log`. Architecture refresh remains NOT_REQUESTED
with the same affected owners; qualification provenance is an additional
affected contract. This verification file remains a new untracked source file
until included in a future commit; no required source is outside the repository.
