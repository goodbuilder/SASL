# Implementation Plan

## Approval status

Approved by the operator on 2026-09-02. Implementation is authorized only for the documented
read-only GNSS-derived Telemetry vertical slice.

## Slices and dependencies

1. Pure contract: new GNSS local-level baseline-center dataclasses/transform and independent NED
   sentinel tests.
2. GPS adapter/readiness: WGS84/MSL -> existing local tangent convention; missing/degenerate,
   age/future, 1000 ms spread, low-fix evidence; scalar Rg compatibility.
3. Controller mapping: one snapshot subtree using selected P1/P2 and Drone source, with exception
   isolation and no worker/state mutation.
4. UI: fixed one-row table, bounded reason/static contract, exact formatting and tooltips.
5. Integration/docs: existing disparity consumes same Rg/readiness; README and numbered docs.
6. Verification: focused after each slice, affected Telemetry/GNSS/UI/coordinate tests, launcher,
   visual dynamic states, full pytest with known environment residuals reported separately.

## File and component map

- `workstation_core/gnss_baseline_center.py`: pure frame/geometry/readiness contract.
- `workstation_core/coordinate_contract.py`: central non-alias frame constants.
- `workstation_core/ideal_disparity.py`: retain API, reuse scalar Rg/readiness if needed.
- `drone_telemetry_controller.py`: publish mapping only.
- `workspaces.py`: add fixed read-only table/group and projection only.
- `tests/test_workstation_telemetry.py`: independent math/readiness/snapshot/UI cases.
- `tests/test_gnss_baseline_center.py`: hand-built NED and GPS/timing boundary oracle.
- `README.md`, `docs/ui/01_USER_WORKFLOW.md`, `03_WORKSPACE_LAYOUT.md`,
  `06_STATUS_MODEL.md`, `09_IMPLEMENTATION_PLAN.md`, `10_DESIGN_AUDIT.md`, and
  `11_DESIGN_AUDIT_RESULT.md` after approval.

## Migration and rollback steps

No migration. Code rollback removes additive mapping/group. No device/file/settings rollback.

## Stopping conditions

- Any requirement to call the GNSS result canonical camera `baseline_center_frd`.
- Need for attitude, optical pose, lever-arm calibration, new settings, recording schema, worker,
  device command, or Measurement gate beyond this approval.
- Existing scalar Rg cannot remain compatible without a visible behavior decision.
- Fixed geometry clips without a recoverable narrow-screen path or live updates move focus/scroll.
