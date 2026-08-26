# Implementation Plan

## Approval status

Approved on 2026-08-26 at 14:03 KST and implemented.

## Slices

1. startup session/intrinsics behavior characterization — complete
2. packet approval — complete
3. remove the two startup calls in `legacy_gui.py` — complete
4. add focused startup/fresh-record/manual-intrinsics tests in `tests/test_stereo.py` — complete
5. update approved StereoWIRIS docs — complete
6. fully restart and verify blank state, preserved old spool, fresh recording ID, manual intrinsics — complete

## File map

`wiris_stereo_distance/legacy_gui.py`, `tests/test_stereo.py`, `docs/README.md`, `docs/usage.md`,
`docs/testing.md`, `UPDATE.md`. No settings schema or data-file mutation.

## Rollback

Restore only the two packet-owned startup calls/tests/docs. Do not reset the dirty worktree.
