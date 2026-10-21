# Qt Designer UI migration

User requested Workstation UI in Qt Designer .ui files and approved the six acceptance criteria with “좋아. 진행해.” on 2026-10-12. This is a new frontend-only packet, not a resumption of unrelated active hardware packets.

Acceptance: A1 explicit coverage; A2 editable Designer forms; A3 edits reflected at runtime without duplicate Python layout; A4 preserved command/input/status/preview/shutdown workflows; A5 resize and coordinate labels; A6 portable checked-in resources and editing instructions.

Scope: existing PyQt5 shell and stable workspace forms. Custom video, plots, dynamic evidence/models remain Python; list their precise boundaries in the coverage table. No new framework or domain/backend behavior.
