"""Operation-scoped disk indexes; never a second live recording owner.

Rows retain producer frame/axis/origin and time units without conversion.
SQLite is scratch state only, removed before generation publication or on
failure. Cache size is fixed; row count does not determine Python heap use.
"""
from __future__ import annotations

from collections.abc import Iterator, Sequence
from contextlib import contextmanager
import pickle
from pathlib import Path
import sqlite3
import tempfile


@contextmanager
def disk_index(parent: Path):
    parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".row-index-", dir=parent) as directory:
        connection = sqlite3.connect(str(Path(directory) / "rows.sqlite"))
        try:
            connection.execute("PRAGMA cache_size=-2048")
            connection.execute("PRAGMA temp_store=FILE")
            connection.execute("PRAGMA journal_mode=OFF")
            yield connection
        finally:
            connection.close()


class DiskRowTable(Sequence):
    """Replayable ordered rows owned by one scratch connection.

    Pickle is used only for bytes created by this object in its private scratch
    database, never for saved artifacts or externally supplied database files.
    It preserves review cell types (datetime, tuple, large integers) exactly.
    """

    def __init__(self, connection: sqlite3.Connection, name: str):
        if not name.isidentifier():
            raise ValueError("invalid scratch table name")
        self.connection = connection
        self.name = name
        self.connection.execute(f'CREATE TABLE "{name}" (ordinal INTEGER PRIMARY KEY, payload BLOB NOT NULL)')
        self.count = 0

    def append(self, row):
        self.connection.execute(
            f'INSERT INTO "{self.name}" (payload) VALUES (?)',
            (pickle.dumps(row, protocol=pickle.HIGHEST_PROTOCOL),),
        )
        self.count += 1

    def extend(self, rows):
        for row in rows:
            self.append(row)

    def __len__(self):
        return self.count

    def __iter__(self) -> Iterator[dict[str, object]]:
        for (payload,) in self.connection.execute(f'SELECT payload FROM "{self.name}" ORDER BY ordinal'):
            yield pickle.loads(payload)

    def __getitem__(self, index):
        if isinstance(index, slice):
            return DiskRowSlice(self, range(*index.indices(len(self))))
        resolved = index if index >= 0 else len(self) + index
        if not 0 <= resolved < len(self):
            raise IndexError(index)
        row = self.connection.execute(
            f'SELECT payload FROM "{self.name}" ORDER BY ordinal LIMIT 1 OFFSET ?', (resolved,)
        ).fetchone()
        return pickle.loads(row[0])

    def replace(self, index, row):
        resolved = index if index >= 0 else len(self) + index
        if not 0 <= resolved < len(self):
            raise IndexError(index)
        self.connection.execute(
            f'UPDATE "{self.name}" SET payload=? WHERE ordinal=(SELECT ordinal FROM "{self.name}" ORDER BY ordinal LIMIT 1 OFFSET ?)',
            (pickle.dumps(row, protocol=pickle.HIGHEST_PROTOCOL), resolved),
        )

    def delete(self, index):
        resolved = index if index >= 0 else len(self) + index
        if not 0 <= resolved < len(self):
            raise IndexError(index)
        self.connection.execute(
            f'DELETE FROM "{self.name}" WHERE ordinal=(SELECT ordinal FROM "{self.name}" ORDER BY ordinal LIMIT 1 OFFSET ?)',
            (resolved,),
        )
        self.count -= 1


class DiskRowSlice(Sequence):
    def __init__(self, rows, indices):
        self.rows = rows
        self.indices = indices

    def __len__(self):
        return len(self.indices)

    def __getitem__(self, index):
        if isinstance(index, slice):
            return DiskRowSlice(self.rows, self.indices[index])
        return self.rows[self.indices[index]]

    def __iter__(self):
        if self.indices.step == 1 and isinstance(self.rows, DiskRowTable):
            for (payload,) in self.rows.connection.execute(
                f'SELECT payload FROM "{self.rows.name}" ORDER BY ordinal LIMIT ? OFFSET ?',
                (len(self.indices), self.indices.start),
            ):
                yield pickle.loads(payload)
        else:
            for index in self.indices:
                yield self.rows[index]
