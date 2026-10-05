# Mountain/Cloud availability policy

User-authorized change: retain mathematically usable results and store quality warnings.
Policy: `mountain-cloud-availability-v1`, new solves only.

The shared Mountain/Cloud controller opts into advisory solver quality. The pure solver's
default strict behavior is preserved for other callers. Rank deficiency and non-finite
rotation/correspondence residuals block; condition and inlier quality thresholds warn.
Capture GNSS finite non-zero baselines are usable despite low/unknown fix quality.
Existing frame identity, K,D, point structure, candidate selection/freeze, Cloud known
excessive frame delta, and live GNSS provider requirements remain in effect.
Cloud still uses the existing latest-valid-position provider for measurement translation;
this change does not admit invalid updates into the shared GNSS cache.

Coordinates reviewed: paired native u/v are undistorted into optical RDF bearings;
relative rotation is right-from-left RDF, baseline is right-minus-left NED, and Left
world pose remains a prior anchor. No formulas or coordinate transforms changed.

Warnings and policy propagate through candidate input hashes/warnings, method status,
MeasurementAttempt and ESTIMATED outcome. New outcome record aliases expose usable,
blocking_reasons, quality_warnings and gate_policy. Canonical checksum inputs remain
unchanged; derived aliases must not be treated as independently authenticated fields.
CSV/XLSX retain existing status/warnings columns and numerical ESTIMATED results.
Existing candidate objects and historical records are not migrated or rewritten.

Verification: 215 tests passed across pose solvers, optical controller integration,
method readiness/evaluation, workbook, readable records and fill-policy UI. JSON
serialization checks both Mountain and Cloud warnings, usability, policy and canonical
checksum. Low GNSS fix, non-finite positions, zero baseline, high condition and rank
deficiency are covered. An additional inlier-quality test checks a finite full-rank fit
below a requested inlier quality target. SciPy reports the installed NumPy-version warning.
Live operator/hardware execution has not been verified. Existing candidates require
Solve again and an existing Measurement requires a new run to freeze the new candidate.
