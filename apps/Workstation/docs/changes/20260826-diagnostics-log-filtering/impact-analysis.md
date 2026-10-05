# Impact Analysis

## Preserved invariants

- Diagnostics는 read-only evidence이며 source of truth나 command owner가 아니다.
- Workspace/Inspector/navigation selection과 device lifecycle은 변하지 않는다.
- raw message, recording audit persistence, FTP lifecycle buffer는 유지한다.
- UI adapter에서 좌표 변환이나 `x/y/z` 해석을 하지 않는다.

## Affected layers and components

- Core: structured event/value objects, strict legacy classifier, bounded sequence store.
- Owner panel: `_log`가 event를 먼저 수집한 뒤 기존 widget/persistence 경로를 그대로 수행.
- Backend facade/protocol: sequence 이후 incremental event batch 조회.
- UI: table model/proxy, filters/search, pause/pending/follow behavior.
- Tests/docs: contract, thread boundary, UI stability, integration, operator guidance.

## Feature dependency, ownership, and coupling changes

event store의 유일 owner는 `WirisStereoDistancePanel`; Diagnostics는 backend facade만 읽는다.
카메라/Pixhawk/GNSS widget internals에 대한 새 직접 접근을 추가하지 않는다. 기존
`diagnostic_text()`는 compatibility/Inspector를 위해 유지한다.

## Worker CPU/GPU resource impact

새 worker/thread/process/GPU 사용은 없다. 2,000개 bounded event와 visible-tab incremental
update만 추가한다. hidden tab은 기존처럼 렌더링하지 않는다. burst 테스트에서 GUI event
processing과 batch 크기를 확인한다.

## Compatibility and migration

설정·파일·schema migration은 없다. structured facade가 없는 backend는 text fallback을
사용한다. 기존 three-widget log와 FTP 탭은 유지해 rollback과 legacy GUI 호환을 보장한다.

## Rollback and recovery

Log page를 기존 `QPlainTextEdit`/`diagnostic_text()`에 되돌려도 device runtime 및 persisted
data는 영향받지 않는다. event store는 in-memory이므로 별도 데이터 rollback이 없다.

## Risks and assumptions

- legacy free text만으로 장비를 완전히 식별할 수 없다. 애매하면 Shared/Unassigned로 표시.
- `_log`의 off-thread queued path에서 event가 중복/유실되지 않도록 object payload 경계를
  독립 테스트해야 한다.
- paused/hidden 기간에 2,000개를 넘으면 오래된 event가 만료된다. silent loss는 금지.
- Workstation 관련 파일에 기존 사용자 수정이 많아 patch 전 overlap을 다시 확인해야 한다.
