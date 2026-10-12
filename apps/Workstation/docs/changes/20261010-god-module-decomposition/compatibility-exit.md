# Named compatibility boundaries and removal criteria

Compatibility preserves supported existing callers during this source refactoring; it must not own business state or become a new development API.

| Boundary | Permitted responsibility | Current production direction | Exit criterion |
|---|---|---|---|
| workspaces.py | Explicit aliases to actual screen/helper owners | MainWindow and leaf panes import workspace_views owners | Remove aliases in a compatibility-breaking release after supported external consumers migrate; retain identity test until then. No new behavior or screen definitions may be added. |
| standalone_gui.py panel surface | Qt host/events and named legacy entrypoints/properties | Application composition creates feature owners and views | Remove each property/delegate when its production consumers use the corresponding feature port. Remove old panel API after import/call inventory has no supported consumers and equivalent workflow/resource tests pass. |
| recording_controller.py | Named public/legacy calls and composition | Recording owners call named collaborators/state/ledger directly | Remove each wrapper after panel/backend/tests supporting external compatibility migrate; never move ledger state back into the facade. |
| measurement_view_controller.py | Qt integration and existing method entrypoints | Planner/capture/geometry/commit/render owners collaborate directly | Remove a wrapper once all supported callers target its owner and independent/headless plus UI tests prove parity. |

The exit audit checks source import/call inventory, not line count. Internal feature modules may not import outer facades or resolve arbitrary attributes through an unrestricted panel/controller proxy. A remaining compatibility alias does not require retaining the old implementation or duplicate state.
