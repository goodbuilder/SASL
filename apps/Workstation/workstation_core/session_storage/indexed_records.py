"""Bounded finalization indexes using existing geometry/time record builders.

No numerical transform is introduced: optical RDF camera origins, public
baseline-center FRD metres, world NED, and source/host ns clocks are preserved.
The scratch database belongs only to this save generation, never to live rows.
"""
from __future__ import annotations

import json
import sqlite3

from Workstation.workstation_core import time_sync_epoch as time_epoch
from Workstation.workstation_core.geometry_context import geometry_context_record_for_terminal_row
from . import canonical, projection
from .disk_index import DiskRowTable


class FinalizationIndexes:
    def __init__(self, connection, *, metadata, session_id, measurement_id):
        self.connection = connection
        self.session_id = session_id
        self.measurement_id = measurement_id
        self.contexts = DiskRowTable(connection, "geometry_context_rows")
        self.epochs = DiskRowTable(connection, "time_sync_epoch_rows")
        connection.execute("CREATE TABLE identities (namespace TEXT, identity TEXT, payload TEXT, PRIMARY KEY(namespace, identity))")
        # Python producers permit exact integers beyond SQLite's signed int64.
        # Decimal text plus numeric collation preserves that existing contract.
        connection.create_collation("exact_integer", lambda left, right: (int(left) > int(right)) - (int(left) < int(right)))
        connection.execute("CREATE TABLE method_sequences (measurement TEXT PRIMARY KEY, sequence TEXT)")
        connection.execute("CREATE TABLE epoch_lookup (side TEXT, start_ns TEXT COLLATE exact_integer, end_ns TEXT COLLATE exact_integer, exclusive_end INTEGER, ssrc TEXT, generation TEXT, epoch_id TEXT)")
        connection.execute("CREATE INDEX epoch_match ON epoch_lookup(side, ssrc, start_ns)")
        self.anchor = {
            "session_started_unix_ns": time_epoch._exact_json_int(metadata.get("session_started_unix_ns")),
            "session_started_monotonic_ns": time_epoch._exact_json_int(metadata.get("session_started_monotonic_ns")),
            "uncertainty_ns": time_epoch._exact_json_int(
                metadata.get("session_time_anchor_uncertainty_ns")
                if metadata.get("session_time_anchor_uncertainty_ns") is not None
                else metadata.get("measurement_time_anchor_uncertainty_ns")
            ),
        }
        self.stus = time_epoch._stus_reference(metadata)

    def _identity(self, namespace, identity, payload=None):
        encoded = None if payload is None else json.dumps(payload, sort_keys=True, ensure_ascii=False, allow_nan=False)
        try:
            self.connection.execute("INSERT INTO identities VALUES (?, ?, ?)", (namespace, identity, encoded))
            return True
        except sqlite3.IntegrityError:
            previous = self.connection.execute("SELECT payload FROM identities WHERE namespace=? AND identity=?", (namespace, identity)).fetchone()[0]
            if payload is not None and previous != encoded:
                raise ValueError(f"{namespace} collision")
            return False

    def generic_rows(self, rows):
        for row in projection._iter_normalized_generic_rows(rows):
            canonical._apply_measurement_package_identity((row,), measurement_id=self.measurement_id)
            yield row

    def camera_rows(self, side, rows):
        active = None
        active_state = None
        active_id = None

        def record(end_ns, end_contract):
            active["end_contract"] = end_contract
            return time_epoch._epoch_record(
                accumulator=active, end_monotonic_ns=end_ns,
                session_id=self.session_id, measurement_package_id=self.measurement_id,
                session_anchor=self.anchor, stus_reference=self.stus,
            )

        def close(end_ns, end_contract):
            if active is None:
                return
            epoch = record(end_ns, end_contract)
            self.epochs.append(epoch)
            if epoch["status"] == "valid":
                self.connection.execute("INSERT INTO epoch_lookup VALUES (?, ?, ?, ?, ?, ?, ?)", (
                    side, str(epoch["start_monotonic_ns"]), None if epoch["end_monotonic_ns"] is None else str(epoch["end_monotonic_ns"]),
                    int(epoch["end_contract"] == "exclusive_transition_boundary"),
                    None if epoch["rtp_ssrc"] is None else str(epoch["rtp_ssrc"]),
                    None if epoch["stream_generation"] is None else str(epoch["stream_generation"]), epoch["time_sync_epoch_id"],
                ))

        for row in self.generic_rows(rows):
            row["time_sync_epoch_id"] = None
            observed_ns = time_epoch._row_monotonic_ns(row)
            state = time_epoch._mapping_state(row)
            if observed_ns is not None and state is not None:
                if active_state != state:
                    close(observed_ns, "exclusive_transition_boundary")
                    active = {
                        "side": side, "status": state[0], "stream_generation": state[1],
                        "rtp_ssrc": state[2], "clock_epoch_id": state[3], "clock_epoch_state": state[4],
                        "start_monotonic_ns": observed_ns, "last_monotonic_ns": observed_ns,
                        "trigger": time_epoch._transition_trigger(active_state, state, has_stus_reference=self.stus is not None),
                        "first_row": dict(row), "end_contract": "last_observed_row",
                    }
                    active_state = state
                    # Epoch identity excludes its eventual end boundary. Use
                    # the exact core encoder so a long epoch needs no row-index list.
                    active_id = record(None, "last_observed_row")["time_sync_epoch_id"]
                elif observed_ns < int(active["last_monotonic_ns"]):
                    yield row
                    continue
                active["last_monotonic_ns"] = observed_ns
                row["time_sync_epoch_id"] = active_id
            yield row
        close(None if active is None else int(active["last_monotonic_ns"]), "last_observed_row_inclusive")

    def pair_rows(self, rows):
        for index, raw in enumerate(rows):
            row = next(self.generic_rows((raw,)))
            # The legacy normalizer supplied these fields before assigning
            # package/terminal identity. Values and insertion order are retained.
            row.setdefault("sample_index", index)
            row.setdefault("session_elapsed_s", None)
            canonical._apply_terminal_record_identities((row,))
            identity = str(row["terminal_record_id"])
            if not self._identity("terminal_record_id", identity):
                raise ValueError(f"duplicate terminal_record_id: {identity}")
            context = geometry_context_record_for_terminal_row(row)
            context_id = str(context["geometry_context_id"])
            if row.get("geometry_context_id") and str(row["geometry_context_id"]) != context_id:
                raise ValueError("geometry_context_id disagrees with static context")
            row["geometry_context_id"] = context_id
            if self._identity("geometry_context_id", context_id, context):
                self.contexts.append(context)
            side_ids = {}
            for side in ("left", "right"):
                observed = time_epoch._optional_int(row.get(f"{side}_app_receive_monotonic_ns"))
                ssrc = time_epoch._optional_int(row.get(f"{side}_rtp_ssrc"))
                generation = time_epoch._optional_int(row.get(f"{side}_stream_generation"))
                matches = []
                if observed is not None and ssrc is not None:
                    matches = self.connection.execute(
                        "SELECT epoch_id FROM epoch_lookup WHERE side=? AND ssrc=? AND start_ns<=? "
                        "AND (end_ns IS NULL OR (exclusive_end=1 AND end_ns>?) OR (exclusive_end=0 AND end_ns>=?)) "
                        "AND (? IS NULL OR generation=?) LIMIT 2",
                        (side, str(ssrc), str(observed), str(observed), str(observed),
                         None if generation is None else str(generation), None if generation is None else str(generation)),
                    ).fetchall()
                side_ids[side] = str(matches[0][0]) if len(matches) == 1 else None
                row[f"{side}_time_sync_epoch_id"] = side_ids[side]
            row["time_sync_epoch_id"] = side_ids["left"] if side_ids["left"] is not None and side_ids["left"] == side_ids["right"] else None
            yield row

    def method_bundle_rows(self, rows, method_tables):
        for raw in rows:
            # Reuse the existing bundle parser, outcome encoders and hashes;
            # only the cross-bundle uniqueness/order ledger moves to disk.
            bundles, outcomes = projection._materialize_method_evaluation_rows((raw,))
            bundle = bundles[0]
            identity = json.dumps((bundle["pair_id"], bundle["request_generation"]))
            if not self._identity("method_evaluation_bundle", identity):
                raise ValueError(f"duplicate method evaluation bundle: {bundle['pair_id']}")
            measurement = str(bundle["measurement_id"])
            sequence = int(bundle["pair_sequence"])
            previous = self.connection.execute("SELECT sequence FROM method_sequences WHERE measurement=?", (measurement,)).fetchone()
            if previous is not None and sequence <= int(previous[0]):
                raise ValueError("method evaluation bundles must have strictly increasing pair_sequence")
            self.connection.execute("INSERT OR REPLACE INTO method_sequences VALUES (?, ?)", (measurement, str(sequence)))
            for method_id, method_rows in outcomes.items():
                method_tables[method_id].extend(self.generic_rows(method_rows))
            canonical._apply_measurement_package_identity((bundle,), measurement_id=self.measurement_id)
            yield bundle
