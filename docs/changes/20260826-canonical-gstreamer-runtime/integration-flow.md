# Integration Flow

## Action-to-postcondition traces

`run.sh` → canonical interpreter gate → common runtime gate → (Legacy only) ROS overlay → app
imports/settings/model → Qt. Camera action → decoder policy → pane → child `sys.executable` →
GStreamer pipeline → frame/status; Auto NVIDIA runtime fault → recorder finalize → software restart.

## Transition tables

| requested | NVIDIA capability | result |
|---|---|---|
| auto | usable | `nvh264dec`, runtime software fallback allowed |
| auto | unavailable | WARN, `avdec_h264`, allowed |
| nvidia | usable | `nvh264dec`, no software fallback |
| nvidia | unavailable | camera start rejected |
| software | any | `avdec_h264` |

## Failure and recovery flows

gi/typelib/required factory failure blocks Qt. Wrong interpreter blocks before ROS/Qt. Missing apt
package prints manual apt command; missing pip package prints canonical venv pip command.

## Cross-feature contracts and dependency direction

앱들은 neutral common checker에 의존하고 checker는 GUI/device code를 import하지 않는다.
Workstation과 Legacy camera copies는 동일 public policy를 소비하지만 서로 import하지 않는다.
