# Change Brief

## Operator outcome

An operator must be able to perform independent Measurements separated by seconds, hours, or days
without creating, reopening, or closing a Session. The same Workstation execution may perform
several sequential Measurements while useful device connections remain live. Evidence is written
to operator-designated storage destinations by data type instead of beneath a date-time Session
folder. After a Measurement ends, the operator may later open the saved outputs and collect the
needed evidence. A grouping object must not own a live writer or block later field work.

## Motivation

The operator reports that intervals between measurements are very long. The current UI describes
Session as a daily/work-run container and requires `Create Session Folder` before Measurement.
Runtime evidence shows one Session created on 2026-08-23 still open on 2026-08-25 with 122 joined
app runs. Its only child, `m000001`, is `RECOVERABLE`, so the current single-mutable-child invariant
prevents creating the next Measurement. This couples later work to recovery of an earlier capture.

## Included scope

- Remove the operator-visible and persistence-owning Session concept for new Measurements.
- Replace the date-time Session root with explicitly configured storage destinations for each data
  type.
- Define how separately stored outputs retain one Measurement's identity and can later be gathered
  without relying on a Session directory.
- Define how a long idle interval, app restart, device disconnect, old incomplete Measurement, and
  starting the next Measurement should appear and behave.
- Preserve or deliberately migrate Calibration, Alignment, evidence, review, and Measurement
  identity contracts that the Session currently owns.

## Excluded scope and non-goals

- Source implementation was approved after the interview at
  `2026-08-26T12:36:43+09:00`; only the recorded sessionless vertical slice is authorized.
- No measurement algorithm, coordinate frame, calibration math, MAVLink, GNSS/RTK, or camera
  protocol change is implied.
- Existing evidence is not deleted or rewritten during design.
- Today's NTRIP credential-priority defect, MAVLink UDP ownership hardening, ATTITUDE interval
  recovery, and Stock-firmware instance configuration were handled separately and are not part of
  this workflow packet.
- Intrinsics metadata compatibility, missing stereo R,t, rover GNSS no-fix, transient Pixhawk
  PARAM-sync status flicker, Qt timer thread-affinity warnings, and long-duration MAVLink soak are
  separate remaining issues. This Session/Measurement proposal must not absorb or conceal them.

## Acceptance summary

Approved: remove Session and its date-time root from every new capture. One configured folder owns
fixed append files by type; native video/image/ULog uses collision-free creation-time filenames.
There is no common persisted membership owner, completeness verdict, package, revision, Parquet,
or review workbook. The operator collects by factual time later. Old/incomplete work and every
storage failure are excluded from Measurement admission; only a live camera/resource transition
may queue start. Existing Session bytes are untouched and file-manager-only.
