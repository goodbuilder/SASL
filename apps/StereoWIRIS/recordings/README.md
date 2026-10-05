# Runtime recordings

This directory is a runtime artifact store, not a source-data directory.
Session CSV, Parquet, JSON, JSONL, spool, and generation files are ignored by
Git. A recording needed as a test fixture must be minimized, anonymized, and
placed under the relevant `tests/fixtures/` directory with its coordinate and
schema contract documented beside it.

Legacy recordings use this directory. Workstation recordings remain under the
ignored `.local/workstation_data/recordings/` root. Neither application reads through
the other application's store.

Coordinate contract: recording schemas retain their producer-declared frame.
No file move or export implies NED, ENU, body FRD, optical RDF, stereo-center,
left-camera, or right-camera conversion.
