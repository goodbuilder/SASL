# Workflow Impact

## Affected operators and workflow IDs

Field operator performing pre-run checkerboard calibration; workflow `CAL-CAPTURE-FEEDBACK-V1`.

## Current workflow

Open Capture, start Preview/Auto, open Advanced to see recognition, later inspect Photo set to
infer how many usable pairs were saved.

## Proposed workflow

Open Capture, confirm zoom, observe Left/Right overlays and board states, capture or run Auto,
then read run and current-set counters without leaving the primary tab. Select the Photo set
explicitly before Compute exactly as today.

## Preserved, added, removed, and reordered steps

Added only primary feedback. No command, prerequisite, wait, confirmation, artifact, or ordering
is removed or automated.

## Decisions, confirmations, handoffs, waits, and artifacts

The worker completion and persisted files remain success evidence. Waiting for frames/NAL/new pose
updates the last status but does not fabricate a capture count.

## Interruption, cancellation, retry, restart, and resume

Stop Auto and late worker completion retain existing behavior. Run counters reset on active root
change or restart; set inventory is reconstructed from existing files. Retry is the existing next
capture candidate.

## Degraded, error, and recovery behavior

Stale/no-frame/OpenCV-unavailable detection is explicit. Unreadable inventory shows `--` and the
error rather than zero. Recovery is Preview restart, fresh board pose, or filesystem repair.

## Completion and next actions

Candidate counts are capture evidence, not Apply readiness. The operator still selects a Photo set,
runs calculation, reviews quality, saves and applies through existing commands.

## Training, operating-procedure, and compatibility impact

No new control. Operators learn the distinction between `Stereo 저장 후보`, per-side K,D-only
images and the later effective calculation subset.

## Workflow-to-screen, command, state, and regression traceability

Existing capture commands -> existing controller/worker -> existing PNG/metadata -> new read-only
snapshot -> primary preview/status. Advanced/manual/FTP/Auto paths remain regression cases.
