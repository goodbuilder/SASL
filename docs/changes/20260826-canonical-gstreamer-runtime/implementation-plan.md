# Implementation Plan

## Approval status

Approved by the user's 2026-08-25/26 implementation request; no conditions.

## Slices and dependencies

1. interpreter/exit-code contract — implemented and software-verified
2. system/pip manifests and doctor — implemented and real-host-verified
3. pure capability/decoder policy — implemented and software-verified
4. launcher/direct entry and app checks — implemented and software-verified
5. camera-start/runtime fallback and child executable — implemented and software-verified
6. docs, focused tests, regression, hardware handoff — docs/regression complete; hardware handoff pending

## File and component map

`requirements/`, `tools/`, `common/python/sasl_common/`, two `run.sh`/entrypoints,
two `yolo_stream.py` copies, camera controller/legacy panel, runtime/launcher/camera tests,
root/StereoWIRIS/Workstation docs.

## Migration and rollback steps

Backup (do not delete) current `.venv`, create official venv, install root requirements, run
deployment/execution/app checks. Roll back code and rename backup venv into place if required.

## Stopping conditions

Stop for any need to change session format, device protocol, reconnect policy, worker budget,
settings schema, or delete/migrate user data.
