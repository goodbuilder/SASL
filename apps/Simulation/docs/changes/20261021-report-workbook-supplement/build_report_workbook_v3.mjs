import fs from "node:fs/promises";
import path from "node:path";
import { createReadStream } from "node:fs";
import { createHash } from "node:crypto";
import readline from "node:readline";
import { FileBlob, SpreadsheetFile } from "@oai/artifact-tool";

async function repoRoot(start) {
  let current = start;
  while (true) {
    try { await fs.access(path.join(current, ".git")); return current; } catch { /* parent */ }
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
const inputPath = path.join(repo, outputRel, "stereo_v1_2_summary_report_ready_v2.xlsx");
const outputPath = path.join(repo, outputRel, "stereo_v1_2_summary_report_ready_v3.xlsx");
const font = "Noto Sans CJK KR";
const navy = "#253B56";
const ink = "#172033";
const teal = "#DDF1EC";
const assert = (condition, message) => { if (!condition) throw new Error(message); };
async function sha256(file) {
  const hash = createHash("sha256");
  for await (const chunk of createReadStream(file)) hash.update(chunk);
  return hash.digest("hex");
}
function parseCsvLine(line) {
  const result=[];let current="",quoted=false;
  for(let i=0;i<line.length;i++){
    const ch=line[i];
    if(ch==='"'){
      if(quoted && line[i+1]==='"'){current+='"';i++;}
      else quoted=!quoted;
    }else if(ch===","&&!quoted){result.push(current);current="";}
    else current+=ch;
  }
  result.push(current);return result;
}
async function* csvRows(file) {
  const reader=readline.createInterface({input:createReadStream(file,{encoding:"utf8"}),crlfDelay:Infinity});
  let headers;
  for await(const line of reader){
    if(!headers){headers=parseCsvLine(line.replace(/^\uFEFF/,""));continue;}
    if(!line)continue;
    const values=parseCsvLine(line);
    assert(values.length===headers.length,"CSV width mismatch");
    yield Object.fromEntries(headers.map((h,i)=>[h,values[i]]));
  }
}
function styleTitle(sheet,address){sheet.getRange(address).format={fill:navy,font:{name:font,size:14,bold:true,color:"#FFFFFF"},rowHeight:32};}
function styleHeader(sheet,address){sheet.getRange(address).format={fill:navy,font:{name:font,size:10,bold:true,color:"#FFFFFF"},rowHeight:32,wrapText:true,horizontalAlignment:"center",verticalAlignment:"center"};}
function styleBody(sheet,address){sheet.getRange(address).format={font:{name:font,size:10,color:ink},rowHeight:25,verticalAlignment:"center"};}
function note(sheet,address,text){sheet.getRange(address).values=[[text]];sheet.getRange(address).format={fill:teal,font:{name:font,size:10,color:ink},wrapText:true,rowHeight:44};}

console.log("STAGE import");
const wb=await SpreadsheetFile.importXlsx(await FileBlob.load(inputPath));
assert(wb.worksheets.items.length===17,"Expected 17 v2 sheets");
const sourceChartCount=wb.worksheets.getItem("05_그래프").charts.items.length;
assert(sourceChartCount===22,"Expected 22 v2 charts");
const sourceMcCount=wb.worksheets.getItem("08_V0_MC분포").getRange("A3").values;
const pathSelection=JSON.parse(await fs.readFile(path.join(analysis,"path_selection.json"),"utf8"));
const predictions=pathSelection.per_calibration_prediction;
assert(predictions.length===75 && pathSelection.selected_path_id==="P2","Unexpected frozen selection");

console.log("STAGE selection sensitivity");
const loo=wb.worksheets.add("15_선정민감도");loo.showGridLines=false;
loo.getRange("A1:I1").merge();loo.getRange("A1").values=[["P2 경로 선정 · 경로별 동일 반복번호 제외 민감도"]];styleTitle(loo,"A1:I1");
loo.getRange("A2:I2").merge();
note(loo,"A2","동결된 G0/Y0/all_data/trial 0 예측 75개에서 같은 반복번호의 A/B/C 3개 값을 경로마다 제외하고 중앙값 점수(m)를 다시 계산합니다. 사후 민감도 진단이며 원래 P2 선택을 변경하지 않습니다.");
loo.getRange("A4:I4").values=[["제외 반복","P0","P1","P2","P3","P4","남은 자료의 1위","P2−P4 (m)","1위−2위 간격 (m)"]];styleHeader(loo,"A4:I4");
const paths=["P0","P1","P2","P3","P4"];
const scoreLists=new Map();
for(let omit=1;omit<=5;omit++){
  const row=4+omit;
  loo.getRange(`A${row}`).values=[[`R0${omit}`]];
  for(let j=0;j<paths.length;j++){
    const values=predictions.filter(p=>p.path_id===paths[j] && p.repetition!==omit).map(p=>p.predicted_position_uncertainty_mean_sigma_m);
    assert(values.length===12,"LOO must leave 12 method×run values");
    const sorted=[...values].sort((a,b)=>a-b);
    const score=(sorted[5]+sorted[6])/2;
    const col="BCDEF"[j];
    loo.getRange(`${col}${row}`).values=[[score]];
    scoreLists.set(`${omit}:${paths[j]}`,score);
  }
  loo.getRange(`G${row}`).formulas=[[`=INDEX($B$4:$F$4,1,MATCH(MIN(B${row}:F${row}),B${row}:F${row},0))`]];
  loo.getRange(`H${row}`).formulas=[[`=D${row}-F${row}`]];
  loo.getRange(`I${row}`).formulas=[[`=SMALL(B${row}:F${row},2)-MIN(B${row}:F${row})`]];
}
styleBody(loo,"A5:I9");loo.getRange("B5:F9").format.numberFormat="0.000000000";loo.getRange("H5:I9").format.numberFormat="0.000000000";
loo.getRange("A12:C12").values=[["P2 1위 횟수","P4 1위 횟수","변경된 1위 횟수"]];styleHeader(loo,"A12:C12");
loo.getRange("A13:C13").formulas=[["=COUNTIF(G5:G9,\"P2\")","=COUNTIF(G5:G9,\"P4\")","=5-A13"]];styleBody(loo,"A13:C13");
loo.getRange("A15:I15").merge();note(loo,"A15","5개 분석은 서로 독립 비행 실험 5회가 아닙니다. 이번 5개 제외 조건에는 동률이 없었습니다. 동률이 생기면 원래 규칙의 짧은 참조시간→경로 ID를 적용해야 합니다. 다른 MC trial의 순위는 계산하지 않았습니다.");
for(const [col,width] of [["A",19],["B",20],["C",20],["D",20],["E",20],["F",20],["G",22],["H",23],["I",24]])loo.getRange(`${col}:${col}`).format.columnWidth=width;

console.log("STAGE run-level V0");
const runGroups=new Map();let selectedMetricRows=0;
for await(const r of csvRows(path.join(analysis,"evaluation_metrics.csv"))){
  if(r.attitude_variant!=="RPY_IMU" || !["A_rtk_independent","B_rtk_joint"].includes(r.method) || !/^G0-P[0-4]-R0[1-5]$/.test(r.calibration_run_id) || !/^G0-V0-R0[1-5]$/.test(r.evaluation_run_id) || !/^Y[0-6]$/.test(r.yaw_case) || !["all_data","matched_200"].includes(r.data_scope))continue;
  assert(r.evaluation_status==="EVALUATED","Unexpected V0 status");
  assert(r.calibration_run_id.slice(-3)===r.evaluation_run_id.slice(-3),"Calibration/V0 repetition mismatch");
  selectedMetricRows++;
  const pathId=r.calibration_run_id.split("-")[1],rep=Number(r.calibration_run_id.slice(-2));
  const key=[pathId,rep,r.method,r.data_scope].join("|");
  let g=runGroups.get(key);
  if(!g){g={pathId,rep,method:r.method,scope:r.data_scope,calibrationRun:r.calibration_run_id,evaluationRun:r.evaluation_run_id,count:0,sumRMSE:0,sumP95:0,seen:new Set()};runGroups.set(key,g);}
  const caseId=`${r.yaw_case}:${r.trial_index}`;
  assert(!g.seen.has(caseId),`Duplicate MC case ${key}:${caseId}`);g.seen.add(caseId);
  g.count++;g.sumRMSE+=Number(r.position_3d_rmse_m);g.sumP95+=Number(r.position_3d_p95_m);
}
assert(runGroups.size===100 && selectedMetricRows===70000,"Expected 100 run groups × 700 cases");
const trackMap=new Map(wb.worksheets.getItem("02_경로추종").getRange("A6:E44").values.map(r=>[r[0],Number(r[4])]));
const run=wb.worksheets.add("16_실행별V0");run.showGridLines=false;
run.getRange("A1:N1").merge();run.getRange("A1").values=[["V0 위치오차 · 경로별 5개 실행"]];styleTitle(run,"A1:N1");
run.getRange("A2:N2").merge();note(run,"A2","G0 existing-yaw Y0–Y6 × MC 100을 각 교정 run·대응 V0 run쌍 안에서 평균한 기술값입니다. 700 case는 독립 비행 700회가 아닙니다. 비행 반복 독립성은 검증되지 않았습니다.");
run.getRange("A4:E4").values=[["대응 반복","A P0 RMSE (m)","A P2 RMSE (m)","B P0 RMSE (m)","B P2 RMSE (m)"]];styleHeader(run,"A4:E4");
run.getRange("A5:A9").values=Array.from({length:5},(_,i)=>[`R0${i+1}`]);styleBody(run,"A5:E9");
run.getRange("A21:I21").values=[["반복","방법","범위","P0 RMSE (m)","P2 RMSE (m)","P2−P0 (m)","P0 실제시간 (s)","P2 실제시간 (s)","P2−P0 시간 (s)"]];styleHeader(run,"A21:I21");
run.getRange("A47:K47").values=[["경로","반복","방법","범위","교정 run","V0 run","case 수","3D RMSE 평균 (m)","3D P95 평균 (m)","실제 구간시간 (s)","원본 CSV"]];styleHeader(run,"A47:K47");
const sortedGroups=[...runGroups.values()].sort((a,b)=>a.pathId.localeCompare(b.pathId)||a.rep-b.rep||a.method.localeCompare(b.method)||a.scope.localeCompare(b.scope));
const detailRows=new Map();
const detailData=sortedGroups.map((g,i)=>{
  const row=48+i;detailRows.set([g.pathId,g.rep,g.method,g.scope].join("|"),row);
  assert(g.count===700 && g.seen.size===700,`Expected 700 cases in ${g.calibrationRun}`);
  return[g.pathId,`R0${g.rep}`,g.method,g.scope,g.calibrationRun,g.evaluationRun,g.count,g.sumRMSE/g.count,g.sumP95/g.count,trackMap.get(g.calibrationRun),`${analysisRel}/evaluation_metrics.csv`];
});
run.getRange("A48:K147").values=detailData;styleBody(run,"A48:K147");
run.getRange("H48:J147").format.numberFormat="0.000000";
for(let rep=1;rep<=5;rep++){
  const row=4+rep;
  const keys=["P0|A_rtk_independent","P2|A_rtk_independent","P0|B_rtk_joint","P2|B_rtk_joint"];
  const formulas=keys.map(k=>`=H${detailRows.get(`${k.split("|")[0]}|${rep}|${k.split("|")[1]}|all_data`)}`);
  run.getRange(`B${row}:E${row}`).formulas=[formulas];
}
run.getRange("B5:E9").format.numberFormat="0.000000";
const pairRows=[];
for(let rep=1;rep<=5;rep++)for(const method of ["A_rtk_independent","B_rtk_joint"])for(const scope of ["all_data","matched_200"]){
  const row=22+pairRows.length;
  const p0=detailRows.get(`P0|${rep}|${method}|${scope}`),p2=detailRows.get(`P2|${rep}|${method}|${scope}`);
  pairRows.push([`R0${rep}`,method,scope,null,null,null,null,null,null]);
  run.getRange(`D${row}:I${row}`).formulas=[[`=H${p0}`,`=H${p2}`,`=E${row}-D${row}`,`=J${p0}`,`=J${p2}`,`=H${row}-G${row}`]];
}
run.getRange("A22:C41").values=pairRows.map(r=>r.slice(0,3));styleBody(run,"A22:I41");run.getRange("D22:I41").format.numberFormat="0.000000";
run.getRange("A43:I43").merge();note(run,"A43","P2−P0 > 0이면 이 저장된 실행쌍·조건의 P2 결과가 더 큽니다. 반복번호는 대응 V0 run의 식별자일 뿐 독립성 보증이 아닙니다. all_data와 matched_200 순위를 섞지 마세요.");
const chart=run.charts.add("line",run.getRange("A4:E9"));
chart.title="G0 V0 3D RMSE · 5 run쌍 평균 (all_data; MC/yaw는 실행 안에서 집계)";
chart.legend={position:"top",textStyle:{typeface:font}};
chart.yAxis={numberFormatCode:"0.000",numberFormatSourceLinked:false,textStyle:{typeface:font}};
chart.yAxis.title.text="3D RMSE (m)";
chart.setPosition("G4","N18");
for(const [col,width] of [["A",17],["B",22],["C",23],["D",24],["E",26],["F",22],["G",23],["H",25],["I",25],["J",23],["K",92]])run.getRange(`${col}:${col}`).format.columnWidth=width;
run.freezePanes.freezeRows(47);

console.log("STAGE tracking P95 sources");
const p95=wb.worksheets.add("17_추종P95출처");p95.showGridLines=false;
p95.getRange("A1:K1").merge();p95.getRange("A1").values=[["추종 위치 P95 · 기록값과 재정렬값 대조"]];styleTitle(p95,"A1:K1");
p95.getRange("A2:K2").merge();note(p95,"A2","두 값 모두 world ENU [east,north,up]의 GT−참조 3D 거리(m)에 type-7 보간 P95를 사용합니다. 기록값은 flight_state의 저장 GT와 40ms 지연 제한을 사용하고, 재정렬값은 science control 시각에 원시 GT를 다시 보간합니다. V0 재정렬 tracking은 원본 QC에서 숨겨졌습니다.");
p95.getRange("A4:D4").values=[["유효 run","재정렬 P95 존재","양쪽 모두 >1m","재정렬 P95 미제공"]];styleHeader(p95,"A4:D4");
p95.getRange("A5:D5").formulas=[["=COUNTA(A9:A47)","=COUNT(G9:G47)","=COUNTIFS(J9:J47,\"예\",K9:K47,\"예\")","=COUNTBLANK(G9:G47)"]];styleBody(p95,"A5:D5");
p95.getRange("F4:G4").values=[["최대 |차이| (m)","평균 차이 (m)"]];styleHeader(p95,"F4:G4");
p95.getRange("F5:G5").formulas=[["=MAX(N9:N47)","=AVERAGE(I9:I47)"]];styleBody(p95,"F5:G5");
p95.getRange("A8:N8").values=[["run_id","attempt","그룹","경로","기록 P95 (m)","기록 표본 n","재정렬 P95 (m)","재정렬 표본 n","기록−재정렬 (m)","기록 >1m","재정렬 >1m","GT 정렬 JSON · 캠페인 상대","JSON SHA-256","|차이| (m)"]];styleHeader(p95,"A8:N8");
const tracking=wb.worksheets.getItem("02_경로추종").getRange("A6:M44").values;
const trackingByRun=new Map(tracking.map((r,i)=>[r[0],{row:i+6,count:r[10],p95:r[12]}]));
const qcIds=wb.worksheets.getItem("07_사후QC").getRange("A5:B43").values;
const p95Rows=[];
for(const [runId,attemptId] of qcIds){
  const rel=`원본_실행별/${runId}/${attemptId}/gt_alignment_summary.json`;
  const full=path.join(campaign,rel);
  const source=JSON.parse(await fs.readFile(full,"utf8"));
  const t=trackingByRun.get(runId);assert(t,"Missing original tracking row");
  const parts=runId.split("-");const aligned=source.tracking || null;
  p95Rows.push([runId,attemptId,parts[0],parts[1],null,t.count,aligned?.position_3d_p95_m??null,aligned?.valid_gt_aligned_rows??null,null,null,null,rel,await sha256(full),null]);
}
p95.getRange("A9:N47").values=p95Rows;styleBody(p95,"A9:N47");
for(let i=0;i<p95Rows.length;i++){
  const row=9+i,t=trackingByRun.get(p95Rows[i][0]);
  p95.getRange(`E${row}`).formulas=[[`='02_경로추종'!M${t.row}`]];
  p95.getRange(`I${row}`).formulas=[[`=IF(G${row}="","",E${row}-G${row})`]];
  p95.getRange(`J${row}`).formulas=[[`=IF(E${row}>1,"예","아니오")`]];
  p95.getRange(`K${row}`).formulas=[[`=IF(G${row}="","미제공",IF(G${row}>1,"예","아니오"))`]];
  p95.getRange(`N${row}`).formulas=[[`=IF(I${row}="","",ABS(I${row}))`]];
}
p95.getRange("E9:E47").format.numberFormat="0.000000";p95.getRange("G9:G47").format.numberFormat="0.000000";p95.getRange("I9:I47").format.numberFormat="0.000000";p95.getRange("N9:N47").format.numberFormat="0.000000";
p95.getRange("A49:K49").merge();note(p95,"A49","기록 P95: source_evidence/summary_workbook_preparation_001/extract_summary_payload.py:15-34. 재정렬 P95: apps/Simulation/campaigns/stereo_v1_1/align_gt.py:90-119. 이 시트는 원래 QC JSON을 수정하지 않습니다.");
for(const [col,width] of [["A",19],["B",12],["C",11],["D",11],["E",21],["F",17],["G",22],["H",22],["I",24],["J",16],["K",17],["L",70],["M",65],["N",19]])p95.getRange(`${col}:${col}`).format.columnWidth=width;
p95.freezePanes.freezeRows(8);

console.log("STAGE reader context");
const graph=wb.worksheets.getItem("05_그래프");
graph.getRange("A39").values=[["카메라 검출률 · 전체 좌/우 원본 프레임과 연구 구간 명목 stereo pair는 분모가 다름"]];
graph.getRange("A50:D50").merge();note(graph,"A50","좌·우 전체 검출률 = 전체 원본 프레임 중 검출 프레임 / 각 카메라 전체 원본 프레임. 연구 구간 양측 검출률 = 양측 검출 pair / 연구 구간 명목 pair. 100%는 전체 원본 영상 검출률이 아닙니다.");
graph.getRange("A91").values=[["V0 경로별 3D RMSE · G0 5 run쌍×Y0–Y6, G1 3 run쌍×nominal0의 case 평균 (GPS_YAW90 제외)"]];
graph.getRange("A100:E100").merge();note(graph,"A100","G0 P0–P4는 각 5개 대응 run쌍×7 yaw×MC100, G1 P0/P4는 각 3개 run쌍×primary-zero×MC100입니다. 분모가 달라 직접 평균 크기만 비교하지 마세요. 16_실행별V0는 G0의 5개 실행만 제시하며 G1 실행별 값은 포함하지 않습니다. 비행 독립성 미검증.");
graph.getRange("A195").values=[["시간–V0 RMSE · G0 5개 run쌍 case 평균 (all_data, RPY_IMU). P2는 사전 공분산 선택이며 이 실측 시간–오차 비교의 우세점이 아님"]];
graph.getRange("A202:E202").merge();note(graph,"A202","이 all_data 평균에서는 P0가 P2보다 A/B RMSE와 기록된 실제 구간시간 모두 낮습니다. matched_200은 순위가 다릅니다. 실행별 값은 16_실행별V0; 비행 독립성 미검증.");
for(const c of graph.charts.items){
  const title=c.title.text;
  if(title.startsWith("영상 검출률:"))c.title="영상 검출률: 전체 원본 프레임 vs 연구 구간 명목 stereo pair (분모 다름)";
  if(title.startsWith("V0 holdout 3D RMSE by calibration path"))c.title="V0 3D RMSE · G0 5쌍×Y0–Y6 / G1 3쌍×nominal0 (case 평균)";
  if(title.startsWith("Trade-off: 실제 비행시간 vs V0 3D RMSE"))c.title="시간–V0 RMSE: G0 all_data case 평균; P2는 사전 공분산 선택";
}
const map=wb.worksheets.getItem("10_계산추적");
map.getRange("A2").values=[[`경로 기준: 01_실행현황!I, 07_사후QC!P, 09_GT제외프레임!M, 17_추종P95출처!L의 원본_실행별/·파생_재처리/와 98_출처 목록은 캠페인 루트(${campaignRel}/) 기준입니다. 10_계산추적 D열과 13_QC시간간격!K의 apps/Simulation/...는 Git 루트 기준입니다. XLSX만으로 외부 원자료를 다시 열 수는 없습니다.`]];
map.getRange("A2:G2").format.rowHeight=58;
map.getRange("E5").values=[["02_경로추종 M: flight_state의 저장 GT−참조 거리, science 행, GT 시각차 ≤40ms, type-7 P95. 재정렬 P95는 17_추종P95출처에서 별도 비교."]];
map.getRange("A19:G21").values=[
  ["16_실행별V0!A48:K147","G0 5개 경로×5 run×A/B×2 scope의 run 내 평균","Y0–Y6 × MC100 = 각 행 700 case",`${analysisRel}/evaluation_metrics.csv`,"world ENU estimate−truth 3D RMSE/P95; 동일 V0 반복 대응","CSV에서 case mean 고정값; 상단·P0/P2 차이는 셀 수식","물리 반복 독립성은 미검증; 700 case는 700 비행 아님"],
  ["15_선정민감도!B5:I9","반복 1개 제외 P0–P4 사전 점수와 1위","G0 trial-0 75개 예측값; 12개 남김",`${analysisRel}/path_selection.json`,"원 규칙의 중앙값을 5개 제외 조건에 적용한 사후 진단","점수는 고정 Python/JS 동등 집계; winner/delta는 Excel 수식","원래 동결 선택을 변경하지 않음; 다른 MC trial은 미평가"],
  ["17_추종P95출처!A9:N47","기록 GT P95와 재정렬 GT P95 · world ENU m","원본 run 39개; 재정렬 tracking 31개",`${campaignRel}/원본_실행별/<run>/<attempt>/gt_alignment_summary.json; 02_경로추종!M`,"기록 flight_state 저장 GT+40ms 조건 vs science stamp GT 재보간; 둘 다 type-7 P95","E=02 M; I=E−G; J/K는 각각 1m 초과 판정","원본 QC 미수정; V0 재정렬 tracking은 pre-freeze 기록에서 미제공"],
];
map.getRange("A19:G21").format={font:{name:font,size:9,color:ink},wrapText:true,verticalAlignment:"top",rowHeight:55};
wb.worksheets.getItem("99_정의").getRange("B10").values=[["02_경로추종 M/07_사후QC F는 flight_state science 행의 저장 GT−참조 거리(40ms 시각차 제한) P95. 17_추종P95출처의 재정렬값은 같은 science 시각에 원시 GT를 보간해 별도 계산하며 표본·GT 위치가 다를 수 있음. 둘 다 world ENU 3D 거리 m."]];
const summary=wb.worksheets.getItem("00_요약");
const auditRel=`${campaignRel}/source_evidence/post_analysis_verification_001/analysis_results_audit.json`;
const audit=JSON.parse(await fs.readFile(path.join(repo,auditRel),"utf8"));
assert(audit.status==="PASS"&&audit.analysis_id==="analysis_9cfd7c6cd10b"&&audit.scope.includes("no numerical accuracy threshold"),"Unexpected audit contract");
const expectedAuditHash=wb.worksheets.getItem("98_출처").getRange("B6").values[0][0];
assert(await sha256(path.join(repo,auditRel))===expectedAuditHash,"Audit source hash differs from provenance sheet");
summary.getRange("C10").values=[["동결; 범위·스키마·출처 감사 PASS (98_출처 A6). 정확도 합격 판정 아님"]];
summary.getRange("C10").format.wrapText=true;summary.getRange("C10").format.rowHeight=40;

console.log("STAGE recalc and guards");
wb.recalculate();
const looWinners=loo.getRange("G5:G9").values.map(r=>r[0]);
assert(JSON.stringify(looWinners)===JSON.stringify(["P2","P4","P4","P4","P2"]),`Unexpected LOO winners ${looWinners}`);
assert(JSON.stringify(loo.getRange("A13:C13").values[0])===JSON.stringify([2,3,3]),"Unexpected LOO counts");
const p95Summary=p95.getRange("A5:D5").values[0];
assert(p95Summary[0]===39&&p95Summary[1]===31&&p95Summary[2]===31&&p95Summary[3]===8,`Unexpected tracking P95 summary ${p95Summary}`);
assert(wb.worksheets.items.length===20,"Expected 20 sheets");
assert(graph.charts.items.length===sourceChartCount,"Existing chart count changed");
assert(run.charts.items.length===1,"Run-level chart missing");
assert(JSON.stringify(sourceMcCount)===JSON.stringify(wb.worksheets.getItem("08_V0_MC분포").getRange("A3").values),"MC source cell changed");
await fs.mkdir(path.dirname(outputPath),{recursive:true});
const file=await SpreadsheetFile.exportXlsx(wb);await file.save(outputPath);
console.log("OUTPUT="+path.relative(repo,outputPath));
console.log("SHA256="+await sha256(outputPath));
console.log("RUN_GROUPS="+runGroups.size+" SELECTED_CASES="+selectedMetricRows);
