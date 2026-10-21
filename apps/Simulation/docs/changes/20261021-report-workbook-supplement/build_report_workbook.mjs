import fs from "node:fs/promises";
import { createReadStream } from "node:fs";
import { createHash } from "node:crypto";
import path from "node:path";
import readline from "node:readline";
import { FileBlob, SpreadsheetFile } from "@oai/artifact-tool";

async function repoRoot(start) {
  let current = start;
  while (true) {
    try { await fs.access(path.join(current, ".git")); return current; } catch { /* search parent */ }
    const parent = path.dirname(current);
    if (parent === current) throw new Error("SASL Git root not found");
    current = parent;
  }
}

const repo = await repoRoot(process.cwd());
const campaignRel = "apps/Simulation/recordings/stereo_v1_1/paper1_v1_2_001";
const campaign = path.join(repo, campaignRel);
const analysisRel = `${campaignRel}/analysis_versions/analysis_9cfd7c6cd10b`;
const analysis = path.join(repo, analysisRel);
const outputRel = `${campaignRel}/outputs/20261021-postfreeze-summary-review`;
const inputPath = path.join(repo, outputRel, "stereo_v1_2_summary_postfreeze_reviewed.xlsx");
const outputPath = path.join(repo, outputRel, "stereo_v1_2_summary_report_ready_v2.xlsx");
const font = "Noto Sans CJK KR";
const navy = "#253B56";
const ink = "#172033";
const pale = "#EDF3F8";
const teal = "#DDF1EC";
const amber = "#FFF0C2";
const assert = (test, message) => { if (!test) throw new Error(message); };

async function hash(file) {
  const h = createHash("sha256");
  for await (const part of createReadStream(file)) h.update(part);
  return h.digest("hex");
}

function styleTitle(sheet, range) {
  sheet.getRange(range).format = { fill: navy, font: { name: font, size: 14, bold: true, color: "#FFFFFF" }, rowHeight: 30 };
}
function styleHeader(sheet, range) {
  sheet.getRange(range).format = { fill: navy, font: { name: font, size: 10, bold: true, color: "#FFFFFF" }, rowHeight: 30, wrapText: true, verticalAlignment: "center", horizontalAlignment: "center" };
}
function styleBody(sheet, range) {
  sheet.getRange(range).format = { font: { name: font, size: 10, color: ink }, rowHeight: 25, verticalAlignment: "center" };
}
function note(sheet, address, value) {
  sheet.getRange(address).values = [[value]];
  sheet.getRange(address).format = { fill: teal, font: { name: font, size: 10, color: ink }, wrapText: true, rowHeight: 30 };
}
function parseCsvLine(line) {
  const out = []; let current = ""; let quoted = false;
  for (let i = 0; i < line.length; i++) {
    const ch = line[i];
    if (ch === '"') {
      if (quoted && line[i + 1] === '"') { current += '"'; i++; }
      else quoted = !quoted;
    } else if (ch === "," && !quoted) { out.push(current); current = ""; }
    else current += ch;
  }
  out.push(current);
  return out;
}
async function* csvRows(file) {
  const reader = readline.createInterface({ input: createReadStream(file, { encoding: "utf8" }), crlfDelay: Infinity });
  let headers;
  for await (const line of reader) {
    if (!headers) { headers = parseCsvLine(line.replace(/^\uFEFF/, "")); continue; }
    if (!line) continue;
    const values = parseCsvLine(line);
    assert(values.length === headers.length, "CSV column count mismatch");
    yield Object.fromEntries(headers.map((h, i) => [h, values[i]]));
  }
}
const median = (values) => {
  const sorted = [...values].sort((a, b) => a - b);
  if (!sorted.length) return null;
  const mid = Math.floor(sorted.length / 2);
  return sorted.length % 2 ? sorted[mid] : (sorted[mid - 1] + sorted[mid]) / 2;
};

console.log("STAGE import");
const workbook = await SpreadsheetFile.importXlsx(await FileBlob.load(inputPath));
const originalNames = workbook.worksheets.items.map((s) => s.name);
assert(originalNames.length === 13, "Expected 13 input sheets");
const originalReview = workbook.worksheets.getItem("06_리뷰판정");
const originalQc = workbook.worksheets.getItem("07_사후QC");
const originalSelectionRows = originalReview.getRange("A27:F31").values;
const originalQcIds = originalQc.getRange("A5:B43").values;
const graphCount = workbook.worksheets.getItem("05_그래프").charts.items.length;

const selection = JSON.parse(await fs.readFile(path.join(analysis, "path_selection.json"), "utf8"));
const freeze = JSON.parse(await fs.readFile(path.join(analysis, "analysis_freeze.json"), "utf8"));
assert(selection.selected_path_id === "P2" && freeze.selected_path_id === "P2", "Selection ID differs");
assert(selection.per_calibration_prediction.length === 75, "Expected 75 selection predictions");
assert(selection.target_geometry.frame === "world_enu:east,north,up_m", "Unexpected target frame");
assert(freeze.robust_loss.scope === "image residuals only", "Unexpected Huber scope");

console.log("STAGE method contract");
const method = workbook.worksheets.add("11_방법계약");
method.showGridLines = false;
method.getRange("A1:D1").merge(); method.getRange("A1").values = [["분석 방법·입력 계약"]]; styleTitle(method, "A1:D1");
method.getRange("A2:D2").merge();
note(method, "A2", "분석 ID analysis_9cfd7c6cd10b · 아래 설명은 동결 설계 계약입니다. 개별 case 입력 파일의 전수 검증을 뜻하지 않습니다.");
method.getRange("A4:D4").values = [["구분", "입력·추정", "결과 해석", "출처"]]; styleHeader(method, "A4:D4");
const methodRows = [
  ["M0 sensor only", "측정 카메라 RTK 광심, IMU R/P, 지정 yaw 초기값. 영상 교정 없음", "주 비교의 센서 입력 기준선", "analysis_methodology.md:7"],
  ["A RTK independent", "좌우 카메라 자세를 각각 추정. 영상 bbox 중심과 같은 noisy 드론 RTK; RTK 공분산을 영상 잔차 공분산으로 전파", "카메라별 독립 교정", "analysis_methodology.md:8"],
  ["B RTK joint", "영상 pair별 공유 world ENU 잠재점과 noisy 드론 RTK·공분산을 함께 사용", "좌우 공동 교정", "analysis_methodology.md:9"],
  ["C no drone RTK", "영상 pair별 공유 world ENU 잠재점. 드론 RTK·GT·명령 경로를 추정기 입력으로 사용하지 않음", "큰 오차 꼬리·rank 부족 가능성을 함께 해석", "analysis_methodology.md:10,33"],
  ["RPY_IMU", "좌우 카메라의 R/P/Y 6각 추정; 카메라 IMU R/P를 유한 prior로 사용", "자세 변형 비교의 한쪽 조건", "analysis_methodology.md:11"],
  ["YAW_ONLY", "카메라 IMU 평균 R/P 고정; 좌우 yaw 2각만 추정", "RPY_IMU와 같은 IMU R/P 관측 사용", "analysis_methodology.md:12"],
  ["YAW_IDEAL / ORACLE", "참값 보조 진단·기하 참조. 주 A/B/C 비교 밖", "운용 성능으로 일반화 금지", "analysis_methodology.md:13-14"],
  ["MC100 case", "(calibration run, trial index, yaw 조건, data_scope)의 case ID; 한 조건당 100 offline trial", "case 수와 solver 행 수·독립 물리 비행 수를 구분", "analysis_methodology.md:3; MC100_설명.html:126-149"],
  ["공통 합성 입력", "같은 rig 반복·trial의 합성 카메라 RTK/IMU draw를 경로·방법·yaw·대응 V0 비교에 공유하도록 설계", "설계의 paired 조건. 실행별 동일 입력은 trial 파일 해시 감사가 별도 필요", "stereo-calibration-explained.html:213-214"],
  ["표본 범위", "all_data: science 구간의 양안 bbox와 10 Hz 드론 RTK 시각 bracket이 있는 공통 pair mask. C도 이 mask 사용; matched_200은 여기서 시간층화된 고유 200 pair", "C 추정 입력에는 RTK 없음. pair 복제 없음; 부족하면 MATCHED_200_UNMET", "analysis_methodology.md:33; stereo-calibration-explained.html:213-215"],
  ["V0 최종 평가", "P0–P4의 동결된 카메라 자세·광심 + V0 좌우 픽셀로 위치 삼각측량. V0 위치 GT는 평가기에만 사용; 드론 RTK는 V0 측위 입력 아님", "합성 RTK/IMU 생성에는 참 카메라 자세 등을 사용할 수 있음. A/B/C 추정기에는 truth 위치 미입력; V0 자세 재교정·성능 합격 판정 아님", "analysis_freeze.json:evaluation_policy; analysis_methodology.md:27-29,41"],
  ["Huber 영상 손실", `정규화된 영상 잔차에만 Huber 적용; f_scale=${freeze.huber_f_scale}. B RTK·카메라 IMU prior는 공분산 정규화 후 이차 Gaussian`, "A는 RTK 공분산을 픽셀 공분산으로 전파; C에는 드론 RTK 비용 없음", "analysis_freeze.json:robust_loss; analysis_methodology.md:33"],
  ["GPS_YAW90", "noisy 좌우 카메라 RTK baseline 방위각 ±90° 초기값; nominal north는 반구 선택만", "Pixhawk 실측 yaw가 아님. A/B/C 재최적화; yaw prior·고정 없음", "analysis_methodology.md:18-25"],
  ["좌표 계약", "world ENU [east,north,up] m; optical RDF [right,down,forward]; R_WC: optical→world", "Gazebo camera-link FLU quaternion과 PX4 body FRD/local NED를 구분", "analysis_methodology.md:27"],
];
method.getRange("A5:D18").values = methodRows; styleBody(method, "A5:D18");
method.getRange("A5:D18").format.wrapText = true; method.getRange("A5:D18").format.rowHeight = 54;
method.getRange("A14:D15").format.rowHeight = 72;
for (const [column, width] of [["A", 24], ["B", 75], ["C", 60], ["D", 49]]) method.getRange(`${column}:${column}`).format.columnWidth = width;
method.freezePanes.freezeRows(4);

console.log("STAGE selection trace");
const sel = workbook.worksheets.add("12_P2선정추적"); sel.showGridLines = false;
sel.getRange("A1:K1").merge(); sel.getRange("A1").values = [["P2 선정 계산 · 동결 전 공분산 예측"]]; styleTitle(sel, "A1:K1");
sel.getRange("A2:K2").merge();sel.getRange("A3:K3").merge();
note(sel, "A2", "G0 / Y0 / all_data / trial 0 / RPY_IMU / 수렴한 A·B·C만 사용. V0 관측·위치오차는 선택 전에 읽지 않음. 후보는 지정 P0–P4에 한정됩니다.");
note(sel, "A3", "각 교정 실행×방법: V0 타깃 5점의 3D σ를 평균 → 경로별 15개 값의 중앙값을 선택 점수로 사용. 동률은 짧은 3D 참조시간, 그다음 경로 ID.");
sel.getRange("A2:A3").format.rowHeight=45;
sel.getRange("A5:F5").values = [["타깃 N (m)", ...selection.target_geometry.distances_m]];
sel.getRange("A6:F6").values = [["타깃 U (m)", null, null, null, null, null]];
sel.getRange("B6:F6").formulas = [["=B5*TAN(RADIANS(20))","=C5*TAN(RADIANS(20))","=D5*TAN(RADIANS(20))","=E5*TAN(RADIANS(20))","=F5*TAN(RADIANS(20))"]];
sel.getRange("A7:K7").merge();
note(sel, "A7", "모든 타깃은 landing pad reference 기준 world ENU [E=0,N,U=N×tan(20°)] m. σ는 위치 공분산 전파 결과이며 실제 V0 위치오차가 아닙니다.");
sel.getRange("A7").format.rowHeight=45;
const summaryHeader = ["동결 순위", "경로", "선택 점수: 타깃 평균 σ의 중앙값 (m)", "동결 점수 (m)", "차이 (m)", "5점 중 2번째 큰 σ의 중앙값 (m)", "동결 보조값 (m)", "평균 참조시간 (s)", "동결 참조시간 (s)", "실행×방법 수", "선정"];
sel.getRange("A9:K9").values = [summaryHeader]; styleHeader(sel, "A9:K9");
const predictions = [...selection.per_calibration_prediction].sort((a,b) => a.path_id.localeCompare(b.path_id) || a.run_id.localeCompare(b.run_id) || a.method.localeCompare(b.method));
const selectionDetailStart = 30;
const detailHeader = ["경로", "교정 run", "방법", "타깃 평균 σ (m)", "5점 중 2번째 큰 σ (m)", "참조시간 (s)", "원자료 평균 σ", "N600 σ", "N550 σ", "N500 σ", "N450 σ", "N400 σ", "원자료 보조값", "계산 평균 차이", "계산 보조값 차이"];
sel.getRange("A29:O29").values = [detailHeader]; styleHeader(sel, "A29:O29");
const ranges = new Map();
const detailValues = predictions.map((r, i) => {
  const row = selectionDetailStart + i;
  const sigmas = r.predictions_by_target.map((t, j) => {
    assert(t.target_world_enu_m[1] === selection.target_geometry.distances_m[j], "Target N order mismatch");
    assert(Math.abs(t.target_world_enu_m[0]) < 1e-12, "Expected E=0 target");
    return t.predicted_3d_sigma_m;
  });
  assert(sigmas.length === 5, "Expected five targets");
  const old = ranges.get(r.path_id) || [row,row]; old[1]=row; ranges.set(r.path_id,old);
  return [r.path_id,r.run_id,r.method,null,null,r.reference_duration_s,r.predicted_position_uncertainty_mean_sigma_m,...sigmas,r.predicted_position_uncertainty_p95_sigma_m,null,null];
});
sel.getRange(`A${selectionDetailStart}:O${selectionDetailStart+detailValues.length-1}`).values = detailValues;
styleBody(sel, `A${selectionDetailStart}:O${selectionDetailStart+detailValues.length-1}`);
for (let i=0;i<detailValues.length;i++) {
  const row=selectionDetailStart+i;
  sel.getRange(`D${row}`).formulas=[[`=AVERAGE(H${row}:L${row})`]];
  sel.getRange(`E${row}`).formulas=[[`=LARGE(H${row}:L${row},2)`]]; // frozen code's rank-4-of-5 definition
  sel.getRange(`N${row}:O${row}`).formulas=[[`=D${row}-G${row}`,`=E${row}-M${row}`]];
}
const summaryValues = selection.candidate_summary.map((c,i)=>[i+1,c.path_id,null,c.predicted_position_uncertainty_mean_sigma_m,null,null,c.predicted_position_uncertainty_p95_sigma_m,null,c.reference_duration_mean_s,null,c.path_id===selection.selected_path_id?"선정":""]);
sel.getRange("A10:K14").values=summaryValues; styleBody(sel,"A10:K14");
for(let i=0;i<summaryValues.length;i++){
  const row=10+i; const [start,end]=ranges.get(summaryValues[i][1]);
  sel.getRange(`C${row}`).formulas=[[`=MEDIAN(D${start}:D${end})`]];
  sel.getRange(`E${row}`).formulas=[[`=C${row}-D${row}`]];
  sel.getRange(`F${row}`).formulas=[[`=MEDIAN(E${start}:E${end})`]];
  sel.getRange(`H${row}`).formulas=[[`=AVERAGE(F${start}:F${end})`]];
  sel.getRange(`J${row}`).formulas=[[`=COUNT(D${start}:D${end})`]];
}
sel.getRange("A16:K16").merge();sel.getRange("A17:K17").merge();
note(sel,"A16","선택 점수 C열은 아래 75개 코드 산출 타깃 σ에서 Excel AVERAGE→MEDIAN으로 재계산됩니다. D·G·I열은 동결 JSON 고정값이며 E·N·O열은 출처와의 차이를 점검합니다.");
note(sel,"A17","보조값은 5개 타깃 중 두 번째 큰 σ입니다. 동결 코드가 이를 p95라고 저장했지만 일반적인 보간 95백분위가 아닙니다. E열은 LARGE(5점,2)로 원본 연산을 재현합니다.");
sel.getRange("A16:A17").format.rowHeight=45;

sel.getRange("A19:F19").merge();sel.getRange("A19").values=[["방법별 예측 σ · 결합 중앙값이 실제로 반영한 범위"]];
sel.getRange("A19:F19").format={fill:teal,font:{name:font,size:11,bold:true,color:ink},rowHeight:27};
sel.getRange("A20:F20").values=[["경로","A 중앙값 (m)","B 중앙값 (m)","C 중앙값 (m)","결합 선택 점수 (m)","결합−A (m)"]];styleHeader(sel,"A20:F20");
const methodSummaryRows=selection.candidate_summary.map(c=>[c.path_id,null,null,null,null,null]);
sel.getRange("A21:F25").values=methodSummaryRows;styleBody(sel,"A21:F25");
for(let i=0;i<methodSummaryRows.length;i++){
  const row=21+i,pathId=methodSummaryRows[i][0];
  const sourceRows=predictions.map((p,index)=>p.path_id===pathId?{method:p.method,row:selectionDetailStart+index}:null).filter(Boolean);
  for(const [col,name] of [["B","A_rtk_independent"],["C","B_rtk_joint"],["D","C_no_drone_rtk_joint"]]){
    const cells=sourceRows.filter(x=>x.method===name).map(x=>`D${x.row}`);
    assert(cells.length===5,`Expected five ${name} predictions for ${pathId}`);
    sel.getRange(`${col}${row}`).formulas=[[`=MEDIAN(${cells.join(",")})`]];
  }
  sel.getRange(`E${row}`).formulas=[[`=C${10+i}`]];
  sel.getRange(`F${row}`).formulas=[[`=E${row}-B${row}`]];
}
sel.getRange("B21:F25").format.numberFormat="0.000000000";
sel.getRange("A27:K27").merge();
note(sel,"A27","이 자료에서 P0–P4 모두 결합 중앙값=방법 A 중앙값입니다. C의 큰 예측 σ가 중앙값에 반영되지 않았고, B·C 각 방법의 최저 경로는 P4입니다. 선정 규칙 자체와 방법별 우월성은 구분하세요.");
sel.getRange("A27").format.rowHeight=44;
sel.getRange("C10:I14").format.numberFormat="0.000000000";
sel.getRange(`D${selectionDetailStart}:O${selectionDetailStart+detailValues.length-1}`).format.numberFormat="0.000000000";
for(const [col,width] of [["A",17],["B",20],["C",43],["D",24],["E",19],["F",30],["G",20],["H",17],["I",19],["J",17],["K",17],["L",17],["M",19],["N",20],["O",20]])sel.getRange(`${col}:${col}`).format.columnWidth=width;
sel.freezePanes.freezeRows(29);

console.log("STAGE QC cadence");
const qc = workbook.worksheets.add("13_QC시간간격"); qc.showGridLines=false;
qc.getRange("A1:J1").merge();qc.getRange("A1").values=[["연구 구간 수집·시뮬레이션 시간 간격"]];styleTitle(qc,"A1:J1");
qc.getRange("A2:J2").merge();
note(qc,"A2","선택된 39개 캠페인 유효 attempt(counted_in_campaign=true)의 원본 qc_report.json에서 추출. 구형 counted_in_39=false는 workbook 캠페인에서 의도된 별도 플래그입니다. 30 ms는 진단 경고, 카메라 간격 한계는 75 ms입니다.");
qc.getRange("A2").format.rowHeight=45;
qc.getRange("A4:D4").values=[["유효 attempt", "연구구간 완전 커버", "full reference slot 누락 합", "30 ms 진단 초과"]];styleHeader(qc,"A4:D4");
qc.getRange("A5:D5").formulas=[["=COUNTA(A10:A48)","=COUNTIF(H10:H48,\"예\")","=SUM(E10:E48)","=COUNTIF(G10:G48,\"예\")"]];styleBody(qc,"A5:D5");
qc.getRange("F4:H4").values=[["최대 simulation gap (ms)","좌 카메라 최대 gap (ms)","우 카메라 최대 gap (ms)"]];styleHeader(qc,"F4:H4");
qc.getRange("F5:H5").formulas=[["=MAX(F10:F48)","=MAX(I10:I48)","=MAX(J10:J48)"]];styleBody(qc,"F5:H5");
qc.getRange("A9:L9").values=[["run_id","attempt_id","기대 full slot","커버 full slot","누락 full slot","최대 simulation gap (ms)","30 ms 초과","science window 커버","좌 카메라 최대 gap (ms)","우 카메라 최대 gap (ms)","원본 QC 상대경로","QC SHA-256"]];styleHeader(qc,"A9:L9");
const qcRows=[];
for(const [runId,attemptId] of originalQcIds){
  const rel=`${campaignRel}/원본_실행별/${runId}/${attemptId}/qc_report.json`;
  const file=path.join(repo,rel);
  const report=JSON.parse(await fs.readFile(file,"utf8"));
  assert(report.counted_in_campaign===true && report.counted_in_39===false,`Unexpected QC inclusion flags: ${runId}`);
  const c=report.evidence?.science_setpoint_cadence_recomputed;
  const cap=report.evidence?.v12_science_capture;
  assert(c && cap && cap.cameras.left && cap.cameras.right,`Cadence evidence missing: ${runId}`);
  qcRows.push([runId,attemptId,c.expected_full_reference_slots,c.covered_full_reference_slots,c.missing_full_slot_count,c.max_simulation_gap_s*1000,c.diagnostic_simulation_gap_exceeded?"예":"아니오",c.science_window_covered?"예":"아니오",cap.cameras.left.max_gap_ns/1e6,cap.cameras.right.max_gap_ns/1e6,rel,await hash(file)]);
}
assert(qcRows.length===39,"Expected 39 QC rows");
qc.getRange("A10:L48").values=qcRows;styleBody(qc,"A10:L48");qc.getRange("F10:J48").format.numberFormat="0.000";
for(const [col,width] of [["A",19],["B",13],["C",17],["D",17],["E",18],["F",27],["G",17],["H",22],["I",27],["J",27],["K",91],["L",66]])qc.getRange(`${col}:${col}`).format.columnWidth=width;
qc.freezePanes.freezeRows(9);

console.log("STAGE paired P95");
const paired = workbook.worksheets.add("14_P95짝비교");paired.showGridLines=false;
paired.getRange("A1:N1").merge();paired.getRange("A1").values=[["RPY_IMU − YAW_ONLY · 동일 case P95 차이"]];styleTitle(paired,"A1:N1");
paired.getRange("A2:N2").merge();
note(paired,"A2","같은 calibration/evaluation run·MC trial·yaw 조건·표본 범위에서 양쪽 지표가 비교 가능할 때만 ΔP95를 계산합니다. Δ<0이면 RPY_IMU의 P95가 작습니다. 비교 불가 건수는 전체 분모에 남깁니다.");
paired.getRange("A2").format.rowHeight=45;
paired.getRange("A4:O4").values=[["분기","그룹","경로","범위","방법","전체 case","P95 비교 가능","RPY 낮음","RPY 높음","동일","비교 불가","평균 ΔP95 (m)","중앙 ΔP95 (m)","RPY 낮음 / 비교 가능","출처"]];styleHeader(paired,"A4:O4");
const pairGroups=new Map();let sourceRows=0;
for await(const r of csvRows(path.join(analysis,"attitude_variant_comparisons.csv"))){
  sourceRows++;
  const branch=r.family==="sensitivity"?"민감도":r.condition_id==="GPS_YAW90"?"GPS_YAW90":"기존 yaw";
  const key=[branch,r.rig_group,r.path_id,r.data_scope,r.method].join("|");
  let g=pairGroups.get(key);
  if(!g){g={branch,rig:r.rig_group,path:r.path_id,scope:r.data_scope,method:r.method,total:0,lower:0,higher:0,same:0,notComparable:0,deltas:[]};pairGroups.set(key,g);}
  g.total++;
  const delta=Number(r.p95_delta_rpy_minus_yaw_m);
  if(r.comparison_status==="COMPARABLE_CASE_METRIC" && r.p95_delta_rpy_minus_yaw_m!=="" && Number.isFinite(delta)){
    g.deltas.push(delta);
    if(r.p95_direction==="LOWER")g.lower++;
    else if(r.p95_direction==="HIGHER")g.higher++;
    else if(delta===0)g.same++;
    else throw new Error(`P95 direction unavailable for comparable ${key}`);
  }else g.notComparable++;
}
assert(sourceRows===277200,`Unexpected paired comparison row count ${sourceRows}`);
const pairOrder={"기존 yaw":0,"GPS_YAW90":1,"민감도":2};
const sortedGroups=[...pairGroups.values()].sort((a,b)=>pairOrder[a.branch]-pairOrder[b.branch]||a.rig.localeCompare(b.rig)||a.path.localeCompare(b.path)||a.scope.localeCompare(b.scope)||a.method.localeCompare(b.method));
const pairData=sortedGroups.map(g=>[g.branch,g.rig,g.path,g.scope,g.method,g.total,g.deltas.length,g.lower,g.higher,g.same,g.notComparable,g.deltas.length?g.deltas.reduce((a,b)=>a+b,0)/g.deltas.length:null,median(g.deltas),null,`${analysisRel}/attitude_variant_comparisons.csv`]);
const pairEnd=4+pairData.length;
paired.getRange(`A5:O${pairEnd}`).values=pairData;styleBody(paired,`A5:O${pairEnd}`);
for(let row=5;row<=pairEnd;row++)paired.getRange(`N${row}`).formulas=[[`=IF(G${row}=0,"n.a.",H${row}/G${row})`]];
paired.getRange(`L5:M${pairEnd}`).format.numberFormat="0.000000";
paired.getRange(`N5:N${pairEnd}`).format.numberFormat="0.0%";
for(const [col,width] of [["A",17],["B",11],["C",12],["D",18],["E",28],["F",14],["G",18],["H",14],["I",14],["J",11],["K",15],["L",23],["M",25],["N",26],["O",92]])paired.getRange(`${col}:${col}`).format.columnWidth=width;
paired.freezePanes.freezeRows(4);

const trace=workbook.worksheets.getItem("10_계산추적");
trace.getRange("A14:G18").values=[
  ["11_방법계약!A5:D18","A/B/C·자세 변형·MC·V0·Huber·좌표 입력 계약","동결 분석 설계 설명",`${analysisRel}/analysis_methodology.md; ${analysisRel}/analysis_freeze.json`,"추정기 입력과 진단 truth 경계; 설계 계약과 실행별 검증 구분","정적 설명; 수치 재계산 없음","모의 센서 생성은 참 카메라 pose 사용 가능; YAW_IDEAL/ORACLE은 truth 보조 진단"],
  [`12_P2선정추적!H${selectionDetailStart}:L${selectionDetailStart+detailValues.length-1}`,"5개 V0 타깃의 예측 3D 위치 σ · m; world ENU target","75 교정 실행×방법 × 5 타깃",`${analysisRel}/path_selection.json:per_calibration_prediction`,"analyze_campaign.py:1645-1681; 공분산 전파 수치는 동결 코드 고정 결과","Drow=AVERAGE(Hrow:Lrow); Erow=LARGE(Hrow:Lrow,2)","보조값은 5점 중 두 번째 큰 σ; 일반적 보간 P95가 아님"],
  ["12_P2선정추적!B21:F25","방법별 5개 중앙값과 결합 선택 점수 · m","A/B/C 각 5개, 결합 15개",`${analysisRel}/path_selection.json:per_calibration_prediction`,"analyze_campaign.py:1682-1700; score=median(per-calibration target mean); tie=mean duration→path ID","방법별 MEDIAN(5개 상세 D); 결합=C10:C14; F=결합−A","이 자료에서는 5개 경로 모두 결합=A; C의 큰 값은 중앙값에 반영되지 않음"],
  ["13_QC시간간격!A5:L48","39개 캠페인 유효 attempt의 setpoint·카메라 구간 수집","counted_in_campaign=true; 구형 counted_in_39=false",`${campaignRel}/원본_실행별/<run>/<attempt>/qc_report.json`,"science_setpoint_cadence_recomputed; v12_science_capture","요약=COUNTA·COUNTIF·SUM·MAX; 상세=동결 QC 고정값","각 QC SHA-256은 L열; 30ms는 진단 경고, 75ms는 카메라 gap 한계"],
  [`14_P95짝비교!A5:O${pairEnd}`,"같은 case의 ΔP95 = RPY_IMU − YAW_ONLY · m","분기×그룹×경로×범위×방법; 비교 가능 분모",`${analysisRel}/attitude_variant_comparisons.csv`,"comparison_status=COMPARABLE_CASE_METRIC; p95_direction; 비교 불가 별도","고정 집계 평균·중앙값; N=H/G (비교 가능 사례 분모)","전체 case·비교 불가를 함께 표시; C의 극단 오차 주의"],
];
trace.getRange("A14:G18").format={font:{name:font,size:9,color:ink},wrapText:true,verticalAlignment:"top",rowHeight:55};

// Correct two reader-facing labels in the existing review sheet: the displayed candidate
// value is a median of per-calibration target means, despite its frozen JSON key name.
originalReview.getRange("A24").values=[["경로 선정 기준: 각 교정 실행×방법에서 V0 타깃 5점의 예측 3D σ를 평균하고, 경로별 15개 평균의 중앙값을 최소화합니다. 이 자료에서는 결합 중앙값이 A 방법의 중앙값과 같습니다. 5점 중 두 번째 큰 σ는 보조값이며 동률에는 참조시간→경로 ID를 사용합니다. V0 측위오차는 선정에 사용하지 않았습니다."]];
originalReview.getRange("D26").values=[["선택 점수: 타깃 평균 σ의 중앙값 (m)"]];
originalReview.getRange("E26").values=[["5점 중 2번째 큰 σ의 중앙값 (m)"]];
originalReview.getRange("A24:F24").format.rowHeight=54;
originalReview.getRange("D26:E26").format.wrapText=true;

const summary=workbook.worksheets.getItem("00_요약");
summary.getRange("C13").values=[["G0 사전 공분산 점수로 선정; V0 최저 오차 주장 아님"]];
summary.getRange("B14").values=[["지표 산출 완료"]];
summary.getRange("C14").values=[["V0 지표 산출; 정확도 합격 기준 없음"]];
summary.getRange("C13:C14").format.wrapText=true;
summary.getRange("C13:C14").format.rowHeight=40;
workbook.worksheets.getItem("99_정의").getRange("B21").values=[["상세 분석·원시/실패 attempt는 캠페인 폴더에 별도 보관. 이 XLSX만 전달하면 원자료 재분석은 할 수 없음"]];
const graph=workbook.worksheets.getItem("05_그래프");
graph.getRange("A224").values=[["GPS_YAW90 3D RMSE paired comparison: RPY_IMU vs YAW_ONLY (comparable-case denominator; P95 direction is in 14_P95짝비교)"]];
graph.getRange("C225:D225").values=[["3D RMSE 낮음", "3D RMSE 높음"]];
graph.getRange("G225:H225").values=[["3D RMSE 낮음 비율", "3D RMSE 높음 비율"]];
const gpsChart=graph.charts.items.find(c=>c.title.text.startsWith("GPS_YAW90 paired comparison:"));
assert(gpsChart,"GPS_YAW90 paired chart missing");
gpsChart.title="GPS_YAW90 3D RMSE paired comparison: RPY_IMU vs YAW_ONLY (%)";
gpsChart.yAxis.title.text="Share of comparable 3D RMSE cases";

console.log("STAGE recalc");
workbook.recalculate();
const score=sel.getRange("C10:E14").values;
for(let i=0;i<5;i++){
  assert(Math.abs(score[i][0]-score[i][1])<1e-10,`Path score mismatch ${summaryValues[i][1]}`);
  assert(Math.abs(score[i][2])<1e-10,`Path score difference nonzero ${summaryValues[i][1]}`);
}
const byMethod=sel.getRange("B21:F25").values;
for(const [i,row] of byMethod.entries())assert(Math.abs(row[4])<1e-10,`Combined score differs from A for ${summaryValues[i][1]}`);
assert(byMethod[1][1]<byMethod[0][1] && byMethod[1][2]<byMethod[0][2],"Expected P4 below P2 for B and C");
assert(sel.getRange("K10").values[0][0]==="선정" && summaryValues[0][1]==="P2","P2 selected row changed");
const qcTotals=qc.getRange("A5:D5").values[0];
assert(qcTotals[0]===39 && qcTotals[1]===39 && qcTotals[2]===0 && qcTotals[3]===30,`QC totals unexpected: ${qcTotals}`);
assert(qc.getRange("F5:H5").values[0][0]===36,"QC max gap unexpected");
for(const g of pairGroups.values())assert(g.total===g.lower+g.higher+g.same+g.notComparable,`Paired P95 total mismatch ${g.method}`);
assert(workbook.worksheets.getItem("05_그래프").charts.items.length===graphCount,"Original charts changed");
assert(JSON.stringify(originalQc.getRange("A5:B43").values)===JSON.stringify(originalQcIds),"Original QC IDs changed");
assert(JSON.stringify(originalReview.getRange("A27:F31").values)===JSON.stringify(originalSelectionRows),"Existing selection values changed");
assert(workbook.worksheets.items.length===originalNames.length+4,"Sheet count unexpected");

await fs.mkdir(path.dirname(outputPath),{recursive:true});
const file=await SpreadsheetFile.exportXlsx(workbook);
await file.save(outputPath);
console.log("OUTPUT="+path.relative(repo,outputPath));
console.log("SHA256="+await hash(outputPath));
console.log("PAIR_GROUPS="+pairData.length);
