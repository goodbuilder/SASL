# Design

## Source of truth

- Status: Active
- Last refreshed: 2026-07-24
- Primary product surface: the tabbed `OperatorPanel` in the default rqt window.
- Evidence reviewed: `Simulation/config/operator_rqt.perspective`, `Simulation/stereo_ground_sim/stereo_ground_sim/operator_panel.py`, `Simulation/stereo_ground_sim/stereo_ground_sim/recorder_panel.py`, `Simulation/run_operator_view.sh`, `Simulation/docs/RQT_OPERATOR_GUIDE.md`, live rqt captures.

## Brand

- Personality: operational, compact, evidence-first.
- Trust signals: explicit FRD units, live topic status, one unambiguous recording action.
- Avoid: dashboard-like decoration, duplicated actions, hidden coordinate conventions.

## Product goals

- Goals: separate setup, live observation, analysis, and raw diagnostics without multiple rqt windows; prioritize readable FRDR plots; retain paired camera verification without letting it dominate the display.
- Non-goals: use rqt as a calibration workflow, expose every recording preset, or make automated flight-quality judgments for the operator.
- Success signals: one tab clearly contains the recording action and pre-record selections; one tab contains camera and live facts; one tab contains full-height comparison plots; raw GPS and GlobalPosition baselines remain visible together.

## Personas and jobs

- Primary personas: simulation operator and flight-test engineer.
- User jobs: verify inputs before flight; monitor stereo and two GPS baselines during flight; start/stop the single CSV record.
- Key contexts of use: wide desktop display with Gazebo/QGC present.

## Information architecture

- Setup: live `/stereo_distance_node` parameter snapshot, scenario/model/coordinate-contract summary, PX4 baseline-source selection, editable YOLO class/confidence/target-rate controls, output location, and one CSV recording action. Only those named live controls are editable before recording and visibly locked while it runs; all other parameters remain read-only.
- Flight: paired camera feeds and direct live values for truth, raw stereo, corrected stereo, raw GPS baseline, and GlobalPosition baseline.
- Analysis: full-height Forward/Right/Down/Range comparison plots with all five sources. For F/R/D-only GPS baseline messages, Range is their explicit 3D vector magnitude.
- Record: the active recorder path and a compact tail of the actual flushed `telemetry.csv` rows; it never reconstructs or substitutes telemetry values.
- Debug: raw topic/profile/status text for expert inspection; it presents facts rather than warnings or pass/fail conclusions.
- Calibration/Full recording: CLI-only expert workflows, not rqt controls.

## Design principles

- Put each flight phase in its own tab so the active task receives the full window.
- One primary action: the Setup tab has exactly one `Save CSV` action and one Stop action; no legacy calibration/debug controls are shown.
- Present raw measurements and configuration; the user, not UI logic, decides whether they are acceptable.
- Keep the coordinate contract visible once per relevant tab and in documentation; do not repeat it as clutter.

## Visual language

- Color: comparison traces distinguish source by color; each plot has a right-side text legend so color is not the sole identifier.
- Typography: native Qt/rqt controls; concise labels.
- Spacing/layout rhythm: compact native controls; balanced paired camera panes; four equal-height analysis plots filling the tab.
- Motion: none beyond live plot updates.

## Components

- Existing components to reuse: native Qt/rqt plugin shell, `CameraView`, `FrdPlotStack`, and the recorder command builder.
- Changed components: `OperatorPanel` becomes the default tab host; recording controls are reusable in both the panel and standalone RecorderPanel compatibility plugin.
- Variants and states: Setup shows Save CSV, Stop, output directory, status, and source selection; active recording disables the editable controls. Flight, Analysis, and Debug have no recording buttons.

## Accessibility

- Target standard: readable at a desktop distance; controls retain native keyboard focus.
- Contrast/readability: do not rely on trace color alone; topic names and FRD ordering stay documented.

## Responsive behavior

- Supported devices: wide desktop rqt window.
- Layout adaptations: each tab fills the rqt central area; camera panes share the Flight tab; plots receive the full Analysis tab height.

## Interaction states

- Loading: direct fields show `--` until their source publishes.
- Empty: plots remain empty until a baseline sample is computed; never synthesize zero.
- Error: status text and raw Debug facts expose missing publishers without assigning a risk level.
- Success: Save CSV changes Setup status and enables Stop.

## Content voice

- Tone: short operational labels.
- Terminology: use Forward, Right, Down; avoid ambiguous x/y/z in operator labels.

## Implementation constraints

- Framework/styling system: ROS 2 Humble rqt / Python Qt; default display is one custom rqt plugin using native Qt widgets.
- Performance constraints: no image copy or plot conversion in the UI; display arrays are published by the measurement node.
- Compatibility constraints: canonical recorder topics remain stamped `baseline_center_frd`; rqt display arrays are FRD `data[0..2]`.
- Test/screenshot expectations: check tab composition, a single CSV action, raw/GlobalPosition baseline presence in live facts and plots, FRD array ordering, and a live rqt capture.

## Open questions

- [ ] Confirm whether a separate preflight-only launch mode is still useful now that Setup is a first-class tab; owner: flight-test operator; impact: launcher simplification.
