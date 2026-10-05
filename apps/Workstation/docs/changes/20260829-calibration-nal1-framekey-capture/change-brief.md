# Change Brief

## Operator outcome

Calibration Auto capture is eligible only when each checkerboard observation belongs to a
parser-observed H.264 NAL type 1, non-delta access unit. The saved lossless native raster must be
the exact FrameKey carried by that observation. The existing status strip explains whether Auto is
waiting for a NAL1 candidate, an exact native frame, a new board pose, or a timing-eligible pair.

## Motivation

The current 1 ms timestamp-near native lookup is not a stereo synchronization gate and can
theoretically substitute a neighbor. The target WIRIS source is expected to provide non-IDR NAL1
intra pictures rather than useful periodic NAL5 IDRs. Compression provenance, checkerboard
observation, native pixels, and persistence must therefore share one immutable frame identity.

## Included scope

- Classify parser-normalized non-delta AUs and retain positive NAL1 evidence by exact PTS in a
  bounded cache.
- Transfer the positive classification to the decoded native FrameKey by exact PTS.
- Admit Auto checkerboard candidates only from that positive NAL1/non-delta metadata.
- Resolve native pixels by exact FrameKey; never use timestamp-near fallback in Auto mode.
- Preserve the existing 2 s freshness, pose, checkerboard, pair-timing, save, and count gates.
- Persist AU/PTS/FrameKey provenance and expose bounded waiting/recovery states.

## Excluded scope and non-goals

- No `SEIR`, forced-IDR, firmware, camera setting, or global keyframe-only stream change.
- No claim that NAL1 is IDR or random-access integrity proof.
- No change to Preview/YOLO/Measurement/recording cadence, manual/FTP/OZ capture, calibration math,
  thresholds, settings schema, or existing photo sets.
- No tolerance increase and no nearest-frame substitution.

## Acceptance summary

The same positive NAL1/non-delta AU, decoded PTS, FrameKey, checkerboard result, native PNG, and
metadata form one traceable chain. Missing classification, stale/evicted keys, generation mismatch,
and malformed metadata wait for a later candidate without saving a neighbor. Unrelated camera and
manual calibration paths remain usable.
