# Impact Analysis

## Preserved invariants

- OpenCV optical RDF image/K,D/R,t/frame contract
- Left/Right capture ID, signed Right-minus-Left Δt, absolute threshold gate
- 기존 `Max Live pair Δt` 값과 설정 ownership
- timing 거부 시 유효 사진의 K,D-only 보존
- FTP CPTR dispatch, verified exposure, Photo set, manual selection, legacy metadata 계약
- Measurement pairing/state, camera worker/queue, capture concurrency/lifecycle

## Affected layers and components

| Layer | Owner/component | Proposed impact |
| --- | --- | --- |
| Domain contract | `workstation_core/calibration_timing.py` | `HOST_RECEIVE_PROXY` 추가 |
| Pure timing logic | `calibration_controller.py` | Live app-receive parse/delta/gate |
| Persistence | `calibration_controller.py`, `calibration_workers.py` existing path | per-side receive ns/basis/evidence JSON |
| Snapshot | `workstation_core/calibration_input.py` | host proxy precision invariant |
| UI text | `workspaces.py`, `standalone_gui.py` | basis/limitation wording only |
| Tests | timing, snapshot, FTP, worker persistence tests | boundary/negative/integration coverage |
| Product docs | `README.md`, `docs/ui/01_USER_WORKFLOW.md`, `docs/ui/06_STATUS_MODEL.md`, `docs/ui/09_IMPLEMENTATION_PLAN.md` | lasting contract update |

## Feature dependency, ownership, and coupling changes

Camera process remains the only producer of decoded-frame receive time. Calibration remains the only owner of
checkerboard pair eligibility. No new cross-feature private-state access or ownership transfer is introduced;
Calibration consumes existing frame metadata through its current native-frame boundary.

## Worker CPU/GPU resource impact

No new worker, queue, thread, decoder, timer, allocation, copy, or I/O loop. Per capture there are two integer
normalizations and one subtraction/comparison. Existing backpressure and GUI responsiveness are unchanged.

## Compatibility and migration

- Existing JSON/photo sets are immutable and retain their historical class/decision.
- New `HOST_RECEIVE_PROXY` rows retain existing `pair_persistence_allowed` booleans, so the legacy pair-usage
  reader continues to accept/reject them without rewriting.
- Mixed historical/new photo sets may expose multiple timing evidence classes; this is provenance, not a solver gate.
- No settings or path migration and no schema-file rewrite.

## Rollback and recovery

Rollback is the scoped code/docs revert. Newly written metadata remains readable through its explicit
allow/reject fields even by the prior pair-usage reader. A missing app timestamp degrades only that capture to
K,D-only; the operator can capture a new pair after Preview recovers.

## Risks and assumptions

- App receipt includes per-side network, jitter-buffer, decode, and scheduling delay and is not exposure time.
- A moving checkerboard can occupy different physical poses even when host arrival Δt is small; operator guidance
  continues to require a stationary board during capture.
- Native-frame `timestamp_s` is contractually app-receive monotonic. Exact integer metadata is preferred; the
  compatibility float may be normalized to ns only at the owned native-frame boundary.
- Field/hardware acceptance cannot be claimed from unit/fake tests; a real dual-WIRIS capture remains manual evidence.
