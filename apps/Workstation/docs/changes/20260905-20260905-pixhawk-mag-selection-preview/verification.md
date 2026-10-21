# Verification

## Requirement-to-evidence matrix

| Requirement | Evidence | Result |
|---|---|---|
| Five-instance topology predicts in-range candidate while retaining CAN2 visibility | `tests/test_mag_selection_preview.py::test_five_instance_topology_keeps_can2_visible_but_predicts_can1` | PASS |
| Four-instance topology predicts CAN2 at instance 3 | `tests/test_mag_selection_preview.py::test_four_instance_topology_predicts_can2_at_instance_three` | PASS |
| Requested priority and actual device remain distinct | `tests/test_mag_selection_preview.py::test_preview_keeps_requested_priority_edit_separate_from_actual_device` | PASS |
| Existing active-device/ATTITUDE gate remains intact | `tests/test_mag_selection.py` | PASS |
| UI preview integrates without breaking existing Pixhawk workspace | `tests/test_pixhawk_magnetometer_workspace.py` | PASS |
| Shell/liveness boundary remains intact | `tests/test_mavlink_liveness.py`, `tests/test_workstation_pixhawk_gnss.py -k 'shell or serial_control'` | PASS |
| Queued Shell active-device output uses newest PX4 timestamp | `test_px4_shell_vehicle_magnetometer_parser_prefers_latest_uorb_timestamp` | PASS |

## Focused test evidence

```text
PYTHONPATH="../..:..:../../common/python" QT_QPA_PLATFORM=offscreen \
  ../../.venv/bin/python -m pytest -q \
  tests/test_mag_selection_preview.py tests/test_mag_selection.py \
  tests/test_pixhawk_magnetometer_workspace.py tests/test_mavlink_liveness.py
60 passed in 2.15s

PYTHONPATH="../..:..:../../common/python" QT_QPA_PLATFORM=offscreen \
  ../../.venv/bin/python -m pytest -q tests/test_workstation_pixhawk_gnss.py \
  -k 'shell or serial_control'
5 passed, 93 deselected in 1.41s
```

## Regression and application evidence

```text
py_compile workstation_core/px4_mag_manager.py workstation_core/pixhawk_attitude.py \
  standalone_gui.py workspaces.py tests/test_mag_selection_preview.py: PASS
git diff --check: PASS
./run.sh --check: PASS
```

The full project run completed with `1931 passed, 2 skipped, 27 failed`. The failures are in
pre-existing/unrelated baseline-yaw, calibration-capture, camera-time-sync, measurement,
network-role, optical-pose, USB transport, settings, and telemetry expectations; no failure
was reported by the focused MAG preview/selection or Shell boundary tests. The full command was:

```text
PYTHONPATH="../..:..:../../common/python" QT_QPA_PLATFORM=offscreen \
  ../../.venv/bin/python -m pytest -q
```

## Performance, load, and target-hardware evidence

## Feature isolation and contract evidence

## Visual authenticity and dynamic-layout stability evidence

## Visual and hardware/manual evidence

The approved UI behavior is read-only topology/candidate rendering; no new firmware or hardware
mutation was introduced. Real hardware evidence for the surrounding MAG selection workflow was
captured separately: direct P1/P2 `vehicle_magnetometer` and live ATTITUDE checks confirmed the
active-device gate before the UI preview implementation. A final visual screenshot review of
the newly added preview at all target DPI/locales remains unverified.

## Residual conditions

- Full-suite residual failures listed above remain outside this packet's changed contract.
- Radio SERIAL_CONTROL can return partial/stale Shell chunks; actual PX4 active selection remains
  authoritative and is not inferred solely from the candidate preview.
- Physical GPS/MAG removal and CAL_MAG slot remapping remain manual/out of scope.

## Real GPS2-removal and UI priority run — 2026-09-05

After physically removing GPS2 and rebooting both Pixhawks, direct `sensor_mag` readback showed:

```text
P1: instance 0=4395041, 1=396809, 2=8944899, 3=8944387
P2: instance 0=4395041, 1=396809, 2=8944643, 3=8944387
```

The RAW-yaw comparison was:

```text
GPS1: 17.744°
Internal: 20.660°
CAN1: 5.055°
CAN2: 6.656°
```

CAN1 was therefore selected as the smallest-difference candidate. The real Qt Workstation UI
handler was driven with the visible P1/P2 priority payload `CAN1=100, CAN2=1` (other roles=1).
The existing PARAM worker read back and applied:

```text
P1: CAL_MAG0/1/2/3_PRIO = 1/1/100/1
P2: CAL_MAG0/1/2/3_PRIO = 1/1/100/1
```

Independent direct PX4 readback then showed `vehicle_magnetometer.device_id=8944899` on P1 and
`8944643` on P2. Fresh ATTITUDE samples were P1 `-159.972°`, P2 `-167.249°`, wrapped delta
`-7.277°`. During the first run the Workstation UI's Radio Shell active-ID label remained
`VERIFYING` because queued Shell responses intermittently reported an older device; the parser
was then corrected to prefer the newest PX4 uORB timestamp and its focused regression passed.

## 2026-10-12 RCA follow-up — staged edits, PARAM retry, voter guard

### Slice evidence

| Slice | Evidence | Result |
|---|---|---|
| PX4 v1.17 sparse-voter gap prediction | `tests/test_mag_selection_preview.py` covers a disabled lower instance, positive filler priority, and the reserved instance-0 validator | PASS |
| Draft survives the 4 Hz Pixhawk refresh; Discard restores confirmed values; voter-gap Apply is blocked | `tests/test_pixhawk_magnetometer_workspace.py::test_pixhawk_workspace_renders_p1_p2_imu_magnetometer_table` | PASS |
| PARAM_SET retries missing ACK only, stops on value mismatch | `tests/test_workstation_pixhawk_gnss.py::test_param_set_retries_missing_ack_but_not_received_value_mismatch`; `tests/test_mavlink_liveness.py` | PASS |
| Worker rejects voter gap before writes and checks final PARAM readback | `tests/test_workstation_pixhawk_gnss.py::test_mag_apply_worker_blocks_known_v117_voter_gap_before_any_write`; `...::test_mag_apply_worker_checks_fresh_final_priority_readback` | PASS |
| Controller captures side topology and invalidates uncertain readback after partial apply | `tests/test_device_admin_boundaries.py` follow-up cases | PASS |

Focused regression command (automated-fake / offscreen, Python 3.10):

```text
PYTHONPATH="../..:..:../../common/python" QT_QPA_PLATFORM=offscreen \
../../.venv/bin/python -m pytest -p no:cacheprovider -q \
  tests/test_mag_selection_preview.py \
  tests/test_pixhawk_magnetometer_workspace.py \
  tests/test_mavlink_liveness.py \
  tests/test_workstation_pixhawk_gnss.py::test_common_param_receive_loop_preserves_integer_bits_and_unmatched_message \
  tests/test_workstation_pixhawk_gnss.py::test_param_set_retries_missing_ack_but_not_received_value_mismatch \
  tests/test_workstation_pixhawk_gnss.py::test_pixhawk_shell_and_parameter_workers_use_exclusive_command_path \
  tests/test_workstation_pixhawk_gnss.py::test_mag_apply_worker_blocks_known_v117_voter_gap_before_any_write \
  tests/test_workstation_pixhawk_gnss.py::test_mag_apply_worker_checks_fresh_final_priority_readback \
  tests/test_device_admin_boundaries.py::test_mag_apply_failure_invalidates_cached_readback_after_possible_partial_write \
  tests/test_device_admin_boundaries.py::test_voter_gap_prewrite_block_keeps_confirmed_readback_available \
  tests/test_device_admin_boundaries.py::test_parameter_worker_receives_captured_side_topology_for_voter_guard
```

Result: latest run `80 passed in 2.63s`. A preceding run passed 79 tests; the added incomplete-
topology warning assertion initially expected the word `disabled` while the compact UI says `off`.
The assertion was corrected and the final run passed. Earlier focused attempts also exposed a
misplaced `actual_role` assertion and a missing public workspace alias; both were corrected.
Scoped `git diff --check` over the changed MAG/Pixhawk source, tests, and product docs passed.

### Not run / residual boundary

- Full Workstation suite not rerun; the earlier packet's `1931 passed, 2 skipped, 27 failed`
  baseline remains recorded above.
- No visual/manual review of the live 1600×980 UI, locale/font scaling, or concurrent Qt Designer
  changes was performed.
- No connected Pixhawk `ver all`, PARAM write, save, reboot, or hardware voter readback was run.
  The local GUI/worker behavior is automated-fake evidence; installed firmware and live selection
  remain hardware-unverified. The user authorized code changes only.
- Coordinate contract: only scalar `sensor_mag` instance, device ID, and `CAL_MAGn_PRIO` values
  were added to the preview guard. No `x/y/z`, sensor-axis, NED/ENU, FRD, or camera-frame values
  were changed or converted.
