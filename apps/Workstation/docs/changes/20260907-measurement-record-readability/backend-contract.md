# Backend contract

Bind (measurement_id, UTC ns, monotonic ns, file_stamp, run root) once at start. Queue items and artifact jobs retain their original route. JSONL and CSV preserve complete payloads. Durable record journal permits workbook reconstruction. Preserve bounded priority/bulk/retry queues and independent CSV retry; reconcile CSV from JSONL after recoverable failure.
