# Mountain/Cloud capture-request Yaw90 reference

Approved behavior: both methods always use GNSS Yaw90, with P1 Roll/Pitch and
request-time telemetry frozen before RTSP/FTP completion. Both camera poses and
their provenance must be recorded. Implemented without device parameter writes.

The pure `optical_yaw90_reference` builder takes copied selected receiver positions,
P1/P2 attitude references and Reverse. It records absent/non-finite inputs as BLOCKED,
retains low fix/age warnings, and computes two initially parallel camera poses using
the existing branch selector. Left is local origin; right is the NED GPS baseline.
Missing inputs do not prevent images being acquired. Imported images cannot acquire
historical reference inputs from live telemetry; FTP completion carries its request snapshot.

The Mountain/Cloud solve uses only this frozen reference. Capture GNSS is converted
NED→left optical RDF using the assumed Left orientation. The fitted right-from-left R
gives Q_right=Q_left*R^T and OpenCV t=-Q_right^T*(C_right-C_left). Existing prior paths
for other methods are preserved. P1 body FRD→camera virtual FRD alignment and antenna
center approximations are explicit assumptions. Euler display is virtual FRD in degrees.

Candidate and selected-profile records contain the reference and both poses. Optional
canonical MeasurementAttempt pose evidence records actual effective poses and R,t,
including Cloud pair GNSS translation. Empty evidence is omitted for legacy hash
compatibility. Raw telemetry attitude values are unchanged. JSONL and CSV record_json
carry the full evidence; method XLSX section D expands the poses and source values.

Readiness has a dedicated Mountain/Cloud reference row; candidate details distinguish
assumed Left from image-derived Right. No source-selection control or fallback to an
existing stereo R,t is added. Existing candidates require recapture/re-solve; running
measurements retain their frozen inputs.

Verification covers pure coordinate signs, missing/non-finite/vertical-baseline inputs,
aged and low-fix warnings, missing P2 reference, identity initial relative rotation,
no-base-R,t solves for both methods, input mutation after capture, FTP completion and
plain-import reset, recorded candidate poses, Cloud effective pair centers, canonical
round trips, CSV envelope and expanded XLSX evidence. Hardware restart and live operator
capture are pending; tests use real math and Qt with simulated telemetry/images.

Final verification: 279 tests passed across Yaw90, optical controller/session/UI/workers,
measurement methods/evaluation/record/report/journal, workbook/reproduction, readable
records and FTP capture sources using `/usr/bin/python3 -m pytest -q -p no:cacheprovider`.
Three focused checks then passed for the actual XLSX/CSV evidence and nullable Yaw90
provenance on other methods. The installed SciPy/NumPy compatibility warning remains.
`git diff --check` passed. New source/test/document files remain uncommitted in the
workspace. The running hardware application was not restarted by this change.
