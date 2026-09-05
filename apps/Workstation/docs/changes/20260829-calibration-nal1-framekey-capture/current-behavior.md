# Current Behavior

## Version and worktree context

- Baseline commit: `7242452` on 2026-08-29.
- Extensive user-owned changes already overlap GStreamer, camera lifecycle, calibration, docs, and
  tests. Implementation must patch only approved hunks and preserve concurrent packet work.
- Coordinate contract review: native left/right OpenCV pixels remain `u` right, `v` down. No NED,
  ENU, FRD, RDF transform, unit, axis, or sign changes are proposed. App monotonic freshness and the
  existing pair-timing clock remain separate contracts.

## Reproduction

Start Calibration Auto capture with both checkerboards visible. The preview worker returns a copied
frame timestamp and metadata. The timer later calls
`calibration_native_frame_near(timestamp_s, maximum_delta_s=0.001)` before capture.

## Observed frontend behavior

Auto polls at the selected interval, samples generic checkerboard previews, rejects repeated poses,
classifies pair timing, and continues until Stop. It has no current-frame NAL1/non-delta waiting
state or proof.

## Observed backend and persistence behavior

- `h264parse` already provides Annex-B AU alignment and `DELTA_UNIT`. The AU probe maps bytes only
  for existing startup/loss recovery inspection.
- A dormant pipeline-wide delta-unit drop filter exists but the product path fixes it false.
- Native frames already have bounded-ring FrameKeys; preview metadata carries descriptor fields;
  `CameraPane.native_frame_for_key()` already performs exact pin/read.
- Auto discards that exact identity at final selection and performs the internal 1 ms near lookup.
- Capture JSON already stores frame IDs, raster grids, hashes, timing, and pose evidence.

## Documentation, code, test, and runtime conflicts

Existing WIRIS docs distinguish NAL1 non-IDR from NAL5 IDR, but Auto has no per-frame AU gate. The
1 ms internal same-camera lookup is easy to confuse with the user-visible left/right pair gate.
Real dual-camera NAL1 cadence and phase remain field-unverified.
