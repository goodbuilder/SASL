# Decisions

Append accepted decisions chronologically. Supersede entries rather than deleting history.

## 2026-08-29 — Exact FrameKey replaces the Auto 1 ms lookup

- Remove timestamp-near selection from Auto instead of increasing its tolerance.
- Preserve freshness and left/right pair timing as independent gates.
- A missing or evicted key waits for the next candidate; it never selects a neighbor.

## 2026-08-29 — Operational candidate is NAL1 plus non-delta

- Require NAL type 1, exclude NAL5, and require `DELTA_UNIT=false` at the parser-normalized AU.
- Never label this category as IDR or source-media integrity proof.

## 2026-08-29 — Keep other consumers unchanged

- Add bounded frame provenance and gate Calibration Auto only.
- Reject the dormant pipeline-wide keyframe-only filter because it would change Preview, YOLO,
  Measurement, and recording cadence.

## 2026-08-29 — Packet and README approved

- The user explicitly approved `approval-summary.md` and confirmed `./README.md` as the documentation
  target.
- Implementation authority is limited to this packet; the separate app-receive timing packet and
  unrelated dirty work remain outside scope.

## 2026-08-29 — Preserve one candidate across existing preview coalescers

- Implementation review found that a positive NAL1 frame could be replaced by the next P-frame in
  the child frame queue or GUI pending slot before checkerboard submission.
- Reuse the existing bounded slots and prefer the positive candidate until one drain; add no queue,
  worker, or unbounded state. Decoder, YOLO, Measurement, and recording admission remain unchanged.

## 2026-08-29 — Software verification complete

- Focused affected regression: 165 passed.
- Full Workstation regression: 1631 passed, 1 skipped.
- Canonical `run.sh --check`, Python compilation, and whitespace checks passed.
- Real dual-WIRIS NAL1 cadence/pair yield remains the explicitly approved hardware-manual residual.
