# Impact Analysis

Workstation-local view/controller/facade and narrow existing settings owner. Existing camera, MAVLink, GNSS and measurement workers retain ownership; no new threads or sockets. Settings key and all record schemas are unchanged. Reuse core NED azimuth and vertex helpers. Preserve unrelated dirty worktree changes; rollback only this change's hunks, not whole files.
