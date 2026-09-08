# Measurement Method comparison

Approved choices: participating Methods only; keep existing Method-midpoint/P1
horizontal coordinates; display one pair until terminal or one second, then jump
to the latest. Display switching never stops or times out calculation.

The record owner maintains a participant union and latest-pair pointer at registration.
Its bounded results_snapshot copies only one pair's Method rows; no complete history,
file read or geometry calculation is performed. DataStorageController exposes this
without initializing storage or the workbook writer; the backend forwards the read API.

MeasurementComparisonPane renders existing FRD, existing horizontal FRD and WGS84
outputs with per-coordinate availability. It displays partial outcomes immediately,
keeps blocked participants, and never carries old numbers into a new pair. Selection
shows the same row's Quality, all coordinates, R,t and frozen input evidence. Legacy
shared labels/history are not used by this pane; their hidden adapters are retained
for compatibility. Existing primary history and Extrinsics analysis preview remain
separate. Coordinate selection does not change the representative Method or storage.

The visible timer runs at 4 Hz, stops while hidden, and retains column widths,
Method selection and scroll. Run identity changes clear prior values. After Stop,
the latest record can receive late outcomes; an old run never replaces a new run.
The one-second deadline is measured from the displayed pair's arrival on screen,
using monotonic time. If no newer pair exists the same pair continues to update.

Coordinate contract checked: original baseline_center_frd, existing
baseline_center_local_level_frd (Method origin / raw pair P1 horizontal forward),
and WGS84 latitude/longitude degrees with separate ellipsoid/MSL heights. No
coordinate formula, archive schema, worker deadline or saved outcome policy changed.

Verification: record isolation/partial/late results, one-second cursor boundary,
early completion, coordinate modes, MSL partial availability, clearing failures,
read-only selection, hidden timer, scroll preservation and a rendered Qt screenshot.
The broader 166-test run passed across records/journal/report/workbook/reproduction,
coordinate logic/storage/UI, workstation UI and Method evaluation. Further focused
checks cover side-effect-free reads and delegate wiring. These use simulated inputs;
the running hardware application has not been restarted. New source/test/docs remain
uncommitted in the workspace.
