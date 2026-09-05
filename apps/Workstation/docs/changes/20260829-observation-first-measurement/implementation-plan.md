# Implementation Plan

## Approval status

Approved 2026-08-29; implementation requested.

## Slices and dependencies

1. Immediate run admission and source-transition contract.
2. Camera failure isolation, busy/recovery join and non-overwriting segments.
3. Other acquisition-source enablement and transition evidence.
4. Non-gating finalization plus bounded essential persistence retry.
5. UI/status/docs and full regression/hardware matrix.
6. Correct the real-device decoder-preflight rollback: distinguish a rejected no-mutation start
   request from a failed replacement, preserve live Preview only in the former case, and retain
   the existing fail-closed explicit-NVIDIA policy.

## File and component map

Primary owners: `measurement_run_controller.py`, `camera_stream_controller.py`,
`workstation_core/measurement_data_store.py`; existing recording/status/workspace adapters provide
the remaining handoffs.

## Migration and rollback steps

No migration. Additive fields only. Revert this packet's focused source changes to roll back.

## Stopping conditions

Stop for any required algorithm/coordinate change, automatic failed-camera reconnect, unbounded
buffer, Session/package revival, file overwrite, new encoder/decoder or integrity gate.

The 2026-09-02 correction is inside the approved failure-isolation slice. It does not change the
decoder capability decision, worker implementation, camera protocol, recording format, or
Measurement result gates.
