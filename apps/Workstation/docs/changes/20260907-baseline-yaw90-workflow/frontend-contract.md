# Frontend Contract

Append Baseline Yaw90 after Combine. Show live cameras, Center line (default off), R/P rows and GNSS rows (azimuth first). Every row has its own source state/age. Inspector exposes selected evidence. Reverse draft survives polling; Apply/Save are distinct; queued/running/stopping locks edits. Navigation does not start Measurement. Remove the moved controls only.

Review follow-up: Inspector payloads include selection_owner=baseline_yaw90. Local-tab exit
requests clearing that owner only, retaining unrelated Inspector and actual table/candidate
selection state. The pane observes its nearest QScrollArea viewport; Resize events schedule a
coalesced owned zero-delay timer. Hide stops pending work and hidden events schedule no fit.

Query-failure follow-up: validate snapshot containers before replacing rows. Cache only a successful
display snapshot, copy it on failure, and add elapsed monotonic time to each age without moving the
original anchor. QUERY_ERROR is rendered as 조회 오류; no CURRENT freshness survives a failed query.
Never replace numeric values with zero/blank when confirmed values exist. An initially unavailable
row remains --, and Inspector resolves the same explicit row even for an empty successful snapshot.
On failure keep Reverse/draft values and disable Apply/Save until a successful read revalidates them.
