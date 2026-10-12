# Workflow Impact

## Before

The operator had to create an output folder, initialize status, run a separate campaign CLI with `--output-root`, follow logs, invoke reprocessing separately, and reason about failure/retry states. The ordinary operator RQT ran inside an active simulation stack, so launching the owning campaign runner from that panel would conflict with its active-SITL guard.

## After

1. The operator edits the literal v1.1 control fields in `config/simulation.env` and clicks **Refresh Simulation.env** in a standalone RQT window.
2. The panel displays action, run ID, output ID, MC count, relative PX4/YOLO configuration, and the config SHA-256.
3. **Run configured action** confirms flight-bearing actions, then launches `control_cli.py` off the GUI thread. The CLI validates the action, resolves a repo-relative output root, runs preflight, initializes status once, and dispatches plan/run/all/resume/reprocess/QC or the separate legacy sweep.
4. For `all_results`, the serial runner executes manifest slots and stops at the first technical/protocol failure. A later `resume` preserves attempts and continues; it automatically starts analysis only after all 39 source slots are valid.
5. The analysis stage reuses the saved cameras/observations, writes immutable trial outputs, chooses a path from G0 covariance propagation, writes `analysis_freeze.json`, and only then reads V0 position GT for evaluation.
6. The RQT progress line and bounded log show slot counts and attempt events. **Stop safely** sends SIGINT so the current attempt is finalized/preserved. A runtime setup button prepares the isolated pinned YOLO/SciPy/zstd environment from the current `.env` values.

## Recovery and failure behavior

Invalid settings disable Start. Existing SITL, wrong PX4 tag/binary, missing model/runtime, failed G0/G1 live pose checks, insufficient image/GT rates, or inadequate storage stop before a flight action. Run failures keep their attempt and QC reason; valid performance failures are not rerun. A repeated technical failure remains blocked until the cause is repaired. Analysis writes are content-checked and idempotent; a differing result is never overwritten.

## Preserved adjacent workflow

`run_auto_attitude_sweep.sh` and its `SWEEP_*` settings remain a separate `legacy_sweep` action. Campaign `all` does not call it, and the control launcher is a separate perspective so its RQT process survives the headless simulation lifecycle.

## Affected operators and workflow IDs

## Current workflow

## Proposed workflow

## Preserved, added, removed, and reordered steps

## Decisions, confirmations, handoffs, waits, and artifacts

## Interruption, cancellation, retry, restart, and resume

## Degraded, error, and recovery behavior

## Completion and next actions

## Training, operating-procedure, and compatibility impact

## Workflow-to-screen, command, state, and regression traceability
