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
