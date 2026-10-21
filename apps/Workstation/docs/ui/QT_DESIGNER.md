# Workstation Qt Designer editing

Workstation uses the existing PyQt5 runtime to load the checked-in forms in
[`ui/forms`](../../ui/forms) through [`ui/designer.py`](../../ui/designer.py).
The forms are the layout source; there is no generated Python file to rebuild
or edit. Save a form, then restart Workstation to see the change.

## Edit and run

From the SASL repository root:

```bash
designer apps/Workstation/ui/forms/settings_workspace.ui
./apps/Workstation/run.sh
```

Use **Qt 5 Designer** to match PyQt5. Designer is an optional development tool
(Ubuntu package `qttools5-dev-tools`), not a new application dependency. The
application only needs the existing PyQt5 environment.

For a simple check, open `settings_workspace.ui`, change the model browse
button's `text`, save, restart, and open Settings → YOLO. The changed label
must appear. Static captions, margins, spacing, grouping and size policies are
editable. Use layouts instead of fixed positions so resizing still works.

The main window and each page are separate forms: opening `main_window.ui`
shows the shell and runtime insertion areas, not the entire populated app.
Open the corresponding page or component form to edit its controls.

## Form map

| Screen | Forms |
|---|---|
| Window, menus, docks, toolbar container | `main_window.ui` |
| Overview / Protocol / Diagnostics | `overview.ui`, `experiment.ui`, `diagnostics.ui` |
| Inspector | `inspector.ui` |
| Settings: all seven tabs | `settings_workspace.ui` |
| Clock / Telemetry / Base GNSS | `time_sync.ui`, `telemetry.ui`, `base_gnss.ui` |
| Cameras | `camera_workspace.ui`, `camera_*.ui` |
| Measurement | `measurement_workspace.ui`, `measurement_*.ui` |
| Data, capture, ULog, review, target reference, worker recovery | `recording_workspace.ui`, `recording_*.ui` |
| Pixhawk | `pixhawk.ui`, `pixhawk_*.ui` |
| Calibration | `calibration.ui`, `calibration_*.ui` |
| Alignment | `alignment.ui`, `baseline_yaw90.ui`, `optical_pose*.ui` |
| Advanced: four tabs | `advanced.ui` |

## Retained Python surfaces

CameraPreview, AnnotationImageView, EngineeringValue, StatusIndicator and
scientific plots still own their drawing, gestures and live formatting. Their
placement is defined by form containers or promoted widgets. Dynamic model
columns, camera discovery results, method catalogs, worker state and responsive
preview sizing remain runtime code. Hidden compatibility-only measurement
quality/result/WGS84 widgets and recording preflight adapters retain their
existing implementation. The legacy backend panel and native file/message
chooser dialogs are not converted into a second Designer application.

## Binding and runtime contracts

- Keep bound **objectName** values unique and unchanged unless the matching
  Python references are updated. `commandId`, `settingsKey`, board/side IDs,
  and other dynamic properties are application bindings, not visible labels.
- Where present, `styleObjectName` or `legacyObjectName` retains an existing
  stylesheet selector after loading. Change `text`/`title` to edit captions.
- Settings, Inspector and OpticalPose combo boxes use an `optionValues` JSON property for
  semantic values. Adding/removing/reordering entries must also update that
  array so displayed options retain their original meaning.
- Workflow tabs and tables have semantic roles. Moving whole groups and
  changing captions is a layout edit; changing workflow order, adding steps or
  rearranging columns also requires updating the corresponding Python binding.
- Persisted settings, live status text, discovered devices, table models,
  conditional visibility and command availability remain Python-owned. A
  form's sample value is not a new runtime default or calibration input.
- Custom camera/annotation/plot/evidence views populate named containers or
  promoted widgets at runtime. Designer displays their placement; live
  rendering, input gestures and data updates remain in their existing owners.
- Menu and toolbar actions come from the command registry; their runtime
  state and identity stay authoritative there. Navigation contents remain the
  navigation model's responsibility.
- Saved window/dock state can override initial form geometry. That is the
  existing restart behavior, not a second layout definition.

No coordinate conversion moved into a form. Preserve the producer's labels:
camera optical RDF, left-camera stereo translation, PX4/body FRD, raw GNSS
NED, optical baseline-center FRD and GNSS midpoint FRD are distinct contracts.
Unit, axis, side and frame edits require the coordinate review in `AGENTS.md`.

## Verification and portability

From the repository root:

```bash
./apps/Workstation/tools/run_isolated_tests.sh -q \
  apps/Workstation/tests/test_designer_forms.py \
  apps/Workstation/tests/test_designer_device_forms.py \
  apps/Workstation/tests/test_designer_engineering.py \
  apps/Workstation/tests/test_settings_designer_form.py \
  apps/Workstation/tests/test_inspector_designer_form.py \
  apps/Workstation/tests/test_optical_pose_designer_form.py \
  apps/Workstation/tests/test_designer_pixhawk_forms.py
python3 tools/check_portable_paths.py
```

The isolated tests check form loading, unique names, edited form propagation,
bindings and input contracts without reaching equipment. Run affected page
tests and the actual application as well after layout changes. Designer
preview does not verify camera rendering, command completion or shutdown.

Forms resolve relative to the installed Workstation package, independently of
the current working directory and data root. Include every `.ui` and required
resource when copying, packaging or committing Workstation. Temporary
export/migration scripts are not required to run or edit these forms.

A complete device-isolated screenshot matrix can be regenerated with:

```bash
unshare --user --map-root-user --net .venv/bin/python -B \
  apps/Workstation/tools/capture_refactor_views.py --designer-coverage \
  --output /tmp/workstation-designer-capture
```

This runs the real shell with hardware startup disabled, temporary settings and
loopback-only networking. It checks all workspace pages, representative tab
states and clean shutdown at three window sizes.

The [migration ledger](../changes/20261012-qt-designer-ui/verification.md)
records tested scope, exact evidence, retained Python surfaces and unverified
hardware conditions.
