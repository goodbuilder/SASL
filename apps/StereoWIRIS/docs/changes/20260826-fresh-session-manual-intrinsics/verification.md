# Verification

| Requirement | Oracle | Status |
|---|---|---|
| Startup ignores retryable spool | Qt state + unchanged spool hash/path | PASS |
| No empty startup session directory | before/after directory listing | PASS |
| Record creates fresh session | session ID/time anchor/new `.spool` and `video/` parent | PASS |
| Startup does not auto-load/apply intrinsics | method spies + active UI state | PASS |
| Manual load/apply still works | retained explicit recovery/manual tests | PASS |
| Existing save/finalize behavior | recording regression tests | PASS |

Existing direct recovery utility tests may remain because the storage reader is retained; only automatic Legacy
startup invocation is removed.

Exact focused command selected five startup/session/recovery tests and returned `5 passed`. A broader
name filter also selected an unrelated large parameter-tab test whose expected text is affected by the real
`intrinsics_zoom_10.json` in this dirty working directory; it returned `10 passed, 1 failed`. That fixture
isolation failure does not call either removed startup hook and was not changed under this packet.
