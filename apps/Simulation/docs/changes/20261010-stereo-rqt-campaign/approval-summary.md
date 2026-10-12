# Approval Summary

This document explains the proposed modification in plain language. Technical detail remains in the other packet documents.

## What changes and what stays the same

The approved scope adds a standalone RQT campaign controller. A person reads and edits the existing `Simulation.env`, refreshes the settings in RQT, and uses one configured-action button to plan, preflight, run a development slot, run/resume the 39-slot campaign, reprocess results from a separately selected input root, run QC, or invoke the independent legacy sweep. The source campaign outputs remain append-only and attempt-aware.

The controller also prepares missing pinned ROS message and YOLO analysis runtimes, implements the required A/B/C attitude solvers and evaluation pipeline, and produces immutable sensor/trial/calibration/evaluation outputs. V0 position errors stay locked until the G0 covariance-based path-selection rule and algorithm hashes are frozen.

## Operator workflow before and after

Before, the operator had to run several terminal commands and the repository had no RQT campaign button or estimator. After, the operator edits the campaign block in `Simulation.env`, opens the standalone RQT perspective, refreshes, and presses **Run configured action**. A separate **Prepare pinned YOLO runtime** control handles the one-time isolated package setup. The old waypoint/OFAT sweep remains an explicit `legacy_sweep` action.

## Success, delay, rejection, failure, overload, and recovery

Invalid or missing settings disable Start. Preflight checks software pins, the selected YOLO device, actual G0/G1 Gazebo camera transforms, short lossless image capture, synchronized GT rate, and storage estimate. A flight-bearing action asks for a final RQT confirmation. The serial runner stops on the first technical/protocol failure and preserves the attempt; `resume` retries only after QC and failure-cause checks. A safe Stop interrupts the owned child and finalizes the current attempt. A low detection rate, poor tracking, or estimator non-convergence remains a recorded result rather than a rerun condition.

## Data and compatibility effects

The 39 manifest slots and original image/GT streams remain the source data. Derived synthetic sensor inputs and results use separate folders and hashes. Main estimators never receive truth; C never receives drone RTK. The 31 reference XLSX files and existing legacy sweep path are outside this change. The previously recorded Desktop 39-run root is read-only during implementation.

## Performance priorities and allowed degradation

Raw capture remains lossless and serial. The offline 100-trial Monte Carlo is intentionally explicit and may run for an extended period; RQT remains responsive and shows bounded logs and progress. Preflight reserves storage based on measured compressed capture rate. CUDA is the existing default; `YOLO_DEVICE=cpu` together with `YOLO_TORCH_BACKEND=cpu` is a human-selectable fallback.

## Visual and interaction changes, including dynamic-layout stability

The panel uses the existing RQT/Qt style with stable labels, standard buttons, and a read-only bounded log. It does not replace the existing operator tabs or resize on every status update. The visible RQT perspective showed `reprocess`, a direct external input root, a separate repository output root, MC=100, and source status `valid=39, failed=0`. QtTest dispatched a 100-trial `reprocess` through the button and received `ANALYSIS_COMPLETE` with exit 0 using a staged read-only source link. DPI scaling, keyboard/focus/scroll behavior, dynamic resizing, and a native mouse click remain unverified.

## How acceptance will be demonstrated

Acceptance evidence includes 81 passing focused tests, the built and visibly loaded standalone RQT perspective, a 100-trial button-driven reprocess, the independent 20-case legacy sweep plan, a fresh read-only QC pass over all 39 source slots, freeze-before-V0 evidence, 50 one-factor cases, and 36,071 verified checksums. Live G0/G1 no-flight preflight passed. No new DEVELOPMENT or 39-slot flight was started because the existing raw source is valid and v1.1 specifies zero additional Gazebo flights; `all_results`/`resume` collection paths remain available but were not run against a new dataset.

## Cost, risks, assumptions, rollback, and field limits

The current clone still has no repo-relative PX4 checkout; the controller discovers an adjacent exact v1.17.0 SITL build as transient runtime provenance. The pinned `px4_msgs` overlay and isolated YOLO venv are prepared, and CUDA is verified on the RTX 4050. Fresh G0/G1 short captures passed without arming or flight. C can remain ill-conditioned; no synthetic result is described as hardware accuracy. Full 100-trial analysis takes extended CPU time. The RQT runtime-preparation action handles a missing pinned YOLO setup.

## Exact scope and authorization

This scope was explicitly authorized in the user's active goal: “code may also be modified” and the required end state is to edit `Simulation.env`, press an RQT button without an LLM, retain the old sweep, and produce the work-instruction results. That authorization applies to this single v1.1 RQT/campaign/result workflow. It does not authorize hardware flight, deleting historical data, or writing the read-only Desktop source root during this implementation.
