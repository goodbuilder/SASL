import fs from "node:fs/promises";
import crypto from "node:crypto";
import path from "node:path";
import { FileBlob, SpreadsheetFile } from "@oai/artifact-tool";

const inputPath = process.argv[2];
const candidatePath = process.argv[3];
const previewDir = process.argv[4] ?? path.join(path.dirname(candidatePath ?? "."), "graph-previews");
if (!inputPath || !candidatePath) {
  throw new Error("Usage: build_summary_graphs.mjs <input.xlsx> <output.xlsx> [preview-dir]");
}
if (path.resolve(inputPath) === path.resolve(candidatePath)) {
  throw new Error("Input and output must differ; the wrapper replaces the workbook only after export succeeds.");
}
const fontFamily = "Noto Sans CJK KR";
const navy = "#253B56";
const ink = "#172033";
const band = "#E7F0F6";
const line = "#9CB5C9";
const palette = ["#4E79A7", "#F28E2B", "#59A14F", "#B279A2", "#76B7B2", "#9C755F"];

await fs.rm(candidatePath, { force: true });
const workbook = await SpreadsheetFile.importXlsx(await FileBlob.load(inputPath));
const graph = workbook.worksheets.getItem("05_그래프");
const worksheetNamesBefore = workbook.worksheets.items.map(s => s.name);

function lastUsedRow(sheetName) {
  const address = workbook.worksheets.getItem(sheetName).getUsedRange()?.address;
  const match = String(address ?? "").match(/(?:^|:)\$?[A-Z]{1,3}\$?(\d+)$/);
  if (!match) throw new Error(`Could not determine used-range end row for ${sheetName}: ${address}`);
  return Number(match[1]);
}
function lastDataRow(sheetName, firstDataRow = 6) {
  const usedEnd = lastUsedRow(sheetName);
  const values = workbook.worksheets.getItem(sheetName)
    .getRange(`A${firstDataRow}:A${usedEnd}`).values.flat();
  const index = values.findLastIndex(value => value !== null && value !== "");
  if (index < 0) throw new Error(`No source data rows found in ${sheetName}`);
  return firstDataRow + index;
}
const flightLastRow = lastDataRow("02_경로추종");
const observationLastRow = lastDataRow("03_관측품질");
const calibrationLastRow = lastDataRow("04_교정비교");
const calibrationKeyRows = workbook.worksheets.getItem("04_교정비교")
  .getRange(`A6:A${calibrationLastRow}`).values.flat();
const existingYawLastRow = calibrationKeyRows.reduce(
  (last, value, index) => value === "EXISTING_YAW" ? index + 6 : last,
  0,
);
if (!existingYawLastRow) throw new Error("04_교정비교 has no EXISTING_YAW rows.");

// Re-running this command recalculates linked formulas without duplicating charts.
const graphBuildMarker = graph.getRange("A26").values?.[0]?.[0];
if (graphBuildMarker === "경로 비용·추종 품질 (그룹/경로별 실행 평균)") {
  if (graph.charts.items.length !== 21 || graph.tables.items.length !== 2) {
    throw new Error(`Partial summary-graph state found: charts=${graph.charts.items.length}, tables=${graph.tables.items.length}`);
  }
  workbook.recalculate();
  const refreshed = await SpreadsheetFile.exportXlsx(workbook);
  await refreshed.save(candidatePath);
  console.log(JSON.stringify({ status: "ALREADY_APPLIED_AND_RECALCULATED", inputPath, candidatePath, chartCount: graph.charts.items.length }));
  process.exit(0);
}
if (graph.charts.items.length !== 3 || graph.tables.items.length !== 2) {
  throw new Error(`Unexpected starting 05_그래프 structure: charts=${graph.charts.items.length}, tables=${graph.tables.items.length}`);
}

function hashJson(value) {
  return crypto.createHash("sha256").update(JSON.stringify(value)).digest("hex");
}

function tableSignature(sheet) {
  return sheet.tables.items.map(t => `${t.name}:${t.getRange().address}`).sort();
}

function chartSignature(sheet) {
  return sheet.charts.items.map(chart => ({
    title: chart.title.text,
    type: chart.type,
    series: chart.series.items.map(s => ({
      name: s.name,
      formula: s.formula,
      categoryFormula: s.categoryFormula,
    })),
  }));
}

function snapshotSheet(sheet) {
  const used = sheet.getUsedRange();
  return {
    name: sheet.name,
    address: used?.address ?? null,
    values: used?.values ?? [],
    formulas: used?.formulas ?? [],
    tables: tableSignature(sheet),
    charts: chartSignature(sheet),
  };
}

const nonGraphBefore = new Map(
  workbook.worksheets.items
    .filter(s => s.name !== "05_그래프")
    .map(s => [s.name, hashJson(snapshotSheet(s))]),
);
const originalGraphTable1 = graph.getRange("A5:F14").values;
const originalGraphTable2 = graph.getRange("A20:E23").values;
const originalChartSignature = chartSignature(graph).slice(0, 3);
const originalGraphTables = tableSignature(graph);

const runPaths = [
  ["G0-P0", "G0", "P0"], ["G0-P1", "G0", "P1"], ["G0-P2", "G0", "P2"],
  ["G0-P3", "G0", "P3"], ["G0-P4", "G0", "P4"], ["G0-V0", "G0", "V0"],
  ["G1-P0", "G1", "P0"], ["G1-P4", "G1", "P4"], ["G1-V0", "G1", "V0"],
];
const calibrationPaths = [
  ["G0-P0", "G0", "P0"], ["G0-P1", "G0", "P1"], ["G0-P2", "G0", "P2"],
  ["G0-P3", "G0", "P3"], ["G0-P4", "G0", "P4"],
  ["G1-P0", "G1", "P0"], ["G1-P4", "G1", "P4"],
];
const mainMethods = ["A_rtk_independent", "B_rtk_joint"];
const allMethods = [...mainMethods, "C_no_drone_rtk_joint"];
const attitudes = ["RPY_IMU", "YAW_ONLY"];
const v0Methods = [
  ["A_rtk_independent", "A"],
  ["B_rtk_joint", "B"],
  ["C_no_drone_rtk_joint", "C"],
];

function quote(value) {
  return `"${String(value).replaceAll('"', '""')}"`;
}

function sourceRange(sheetName, column, first, last) {
  return `'${sheetName}'!$${column}$${first}:$${column}$${last}`;
}

function averageIfs(sheetName, sourceColumn, first, last, criteria) {
  if (sheetName === "02_경로추종") last = flightLastRow;
  if (sheetName === "03_관측품질") last = observationLastRow;
  if (sheetName === "04_교정비교") last = calibrationLastRow;
  if (sheetName === "04_교정비교" && criteria.some(([column, value]) => column === "A" && value === "EXISTING_YAW")) {
    last = existingYawLastRow;
  }
  const source = sourceRange(sheetName, sourceColumn, first, last);
  const pairs = criteria.map(([column, value]) =>
    `${sourceRange(sheetName, column, first, last)},${quote(value)}`,
  ).join(",");
  return `=IFERROR(AVERAGEIFS(${source},${pairs}),"")`;
}

function averageAcrossRunIds(sourceColumn, group, path) {
  const source = sourceRange("03_관측품질", sourceColumn, 6, observationLastRow);
  const runIdRange = sourceRange("03_관측품질", "A", 6, observationLastRow);
  const prefix = `${group}-${path}-`;
  const runIds = [...new Set(workbook.worksheets.getItem("03_관측품질")
    .getRange(`A6:A${observationLastRow}`).values.flat()
    .filter(value => typeof value === "string" && value.startsWith(prefix)
      && /^R\d+$/.test(value.slice(prefix.length)))
    .sort())];
  if (!runIds.length) throw new Error(`No observation run IDs found for ${group}-${path}`);
  const runMeans = runIds.map(runId => `AVERAGEIF(${runIdRange},${quote(runId)},${source})`);
  return `=IFERROR(AVERAGE(${runMeans.join(",")}),"")`;
}

function columnNumber(column) {
  return [...column].reduce((n, ch) => n * 26 + ch.charCodeAt(0) - 64, 0);
}

// Some 04_교정비교 fields (attitude summaries and GPS_YAW90 rows) do not
// calculate reliably through AVERAGEIFS in the XLSX engine. Resolve the exact
// matching source cells and keep the aggregation as a transparent formula.
const calibrationSheet = workbook.worksheets.getItem("04_교정비교");
const calibrationRows = calibrationSheet.getRange(`A6:AK${calibrationLastRow}`).values;
function averageCaseCells(criteria, metricColumn) {
  const metricIndex = columnNumber(metricColumn) - 1;
  const criteriaIndexes = criteria.map(([column, value]) => [columnNumber(column) - 1, value]);
  const refs = [];
  for (let index = 0; index < calibrationRows.length; index++) {
    const row = calibrationRows[index];
    if (criteriaIndexes.every(([columnIndex, value]) => row[columnIndex] === value)
      && typeof row[metricIndex] === "number" && Number.isFinite(row[metricIndex])) {
      const sheetRow = index + 6;
      refs.push(`'04_교정비교'!$${metricColumn}$${sheetRow}`);
    }
  }
  return refs.length ? `=IFERROR(AVERAGE(${refs.join(",")}),"")` : `=""`;
}

function sumRatio(sheetName, numeratorColumn, denominatorColumn, first, last, criteria) {
  if (sheetName === "04_교정비교") last = calibrationLastRow;
  if (sheetName === "04_교정비교" && criteria.some(([column, value]) => column === "A" && value === "EXISTING_YAW")) {
    last = existingYawLastRow;
  }
  const numerator = sourceRange(sheetName, numeratorColumn, first, last);
  const denominator = sourceRange(sheetName, denominatorColumn, first, last);
  const pairs = criteria.map(([column, value]) =>
    `${sourceRange(sheetName, column, first, last)},${quote(value)}`,
  ).join(",");
  return `=IFERROR(SUMIFS(${numerator},${pairs})/SUMIFS(${denominator},${pairs}),"")`;
}

function colLetter(number) {
  let n = number;
  let out = "";
  while (n > 0) {
    const rem = (n - 1) % 26;
    out = String.fromCharCode(65 + rem) + out;
    n = Math.floor((n - 1) / 26);
  }
  return out;
}

function writeTable({ titleRow, headerRow, title, headers, rows, formulaColumns = {}, numberFormats = {} }) {
  const lastColumn = colLetter(headers.length);
  const bodyStart = headerRow + 1;
  const bodyEnd = bodyStart + rows.length - 1;
  graph.getRange(`A${titleRow}`).values = [[title]];
  graph.getRange(`A${titleRow}`).format = {
    font: { name: fontFamily, size: 11, bold: true, color: navy },
    verticalAlignment: "center",
  };
  graph.getRange(`A${headerRow}:${lastColumn}${headerRow}`).values = [headers];
  if (rows.length) graph.getRange(`A${bodyStart}:${lastColumn}${bodyEnd}`).values = rows;
  graph.getRange(`A${headerRow}:${lastColumn}${headerRow}`).format = {
    fill: navy,
    font: { name: fontFamily, size: 9, bold: true, color: "#FFFFFF" },
    horizontalAlignment: "center",
    verticalAlignment: "center",
    wrapText: true,
  };
  graph.getRange(`A${headerRow}:${lastColumn}${headerRow}`).format.rowHeight = 30;
  if (rows.length) {
    const bodyRange = graph.getRange(`A${bodyStart}:${lastColumn}${bodyEnd}`);
    bodyRange.format = {
      font: { name: fontFamily, size: 9, color: ink },
      verticalAlignment: "center",
      wrapText: false,
    };
    bodyRange.format.borders = { preset: "outside", style: "thin", color: line };
    for (let row = bodyStart; row <= bodyEnd; row++) {
      graph.getRange(`A${row}:${lastColumn}${row}`).format.rowHeight = 18;
      if ((row - bodyStart) % 2 === 0) {
        graph.getRange(`A${row}:${lastColumn}${row}`).format.fill = band;
      }
    }
    graph.getRange(`A${bodyStart}:A${bodyEnd}`).format.horizontalAlignment = "left";
    for (const [column, formulas] of Object.entries(formulaColumns)) {
      graph.getRange(`${column}${bodyStart}:${column}${bodyEnd}`).formulas = formulas.map(f => [f]);
      graph.getRange(`${column}${bodyStart}:${column}${bodyEnd}`).format.horizontalAlignment = "right";
    }
    for (const [column, fmt] of Object.entries(numberFormats)) {
      graph.getRange(`${column}${bodyStart}:${column}${bodyEnd}`).setNumberFormat(fmt);
    }
  }
  return { bodyStart, bodyEnd, lastColumn };
}

const axisStyle = { typeface: fontFamily, fontSize: 8 };
const chartPositions = [];
let nextChartTop = 61;

function addChart(type, data, title, yFormat, yTitle, options = {}) {
  const chart = graph.charts.add(type, data);
  chart.title = title;
  chart.titleTextStyle.fontSize = 11;
  chart.titleTextStyle.typeface = fontFamily;
  chart.hasLegend = options.legend !== false;
  if (chart.hasLegend) chart.legend = { position: "top", textStyle: { typeface: fontFamily, fontSize: 8 } };
  if (type !== "scatter") {
    chart.xAxis = { axisType: "textAxis", textStyle: axisStyle };
  }
  chart.yAxis = { numberFormatCode: yFormat, numberFormatSourceLinked: false, textStyle: axisStyle };
  if (options.minY !== undefined) chart.yAxis.min = options.minY;
  if (options.maxY !== undefined) chart.yAxis.max = options.maxY;
  if (options.xTitle) chart.xAxis.title.text = options.xTitle;
  if (yTitle) chart.yAxis.title.text = yTitle;
  chart.setPosition(`H${nextChartTop}`, `Q${nextChartTop + 16}`);
  chartPositions.push({ title, top: nextChartTop, bottom: nextChartTop + 16, type });
  nextChartTop += 18;
  if (options.colors) {
    chart.series.items.forEach((series, index) => {
      const color = options.colors[index % options.colors.length];
      if (type === "line" || type === "scatter") series.line = { fill: color, style: "solid", width: 2 };
      else series.fill = color;
    });
  }
  return chart;
}

function applyScatterPointLabels(chart, labels, seriesIndices = chart.series.items.map((_, index) => index)) {
  let applied = true;
  for (const seriesIndex of seriesIndices) {
    for (let pointIndex = 0; pointIndex < labels.length; pointIndex++) {
      try {
        chart.applyDataLabels(
          { scope: "point", seriesIndex, pointIndex },
          { text: labels[pointIndex], position: "right", showValue: false, textStyle: { typeface: fontFamily, fontSize: 8 } },
        );
      } catch {
        applied = false;
      }
    }
  }
  return applied;
}

// Broaden the heading and make the data interpretation explicit.
graph.getRange("A2").values = [["비행·관측·교정 성능 그래프"]];
graph.getRange("A3").values = [["V0 수치는 분석 동결 뒤의 홀드아웃 평가입니다. MC 사례 평균은 물리 비행 반복 횟수가 아닙니다."]];
graph.getRange("A3").format = { font: { name: fontFamily, size: 9, italic: true, color: "#58677A" }, wrapText: false };

// Reuse the workbook's scalar summaries. Coordinate meaning stays owned by 99_정의:
// world ENU [east,north,up], PX4 local NED [north,east,down], optical RDF [right,down,forward].
// No axis swaps or frame conversions happen in this graph builder.
// Flight and tracking summaries, averaged across valid run records in the same group/path.
writeTable({
  titleRow: 26, headerRow: 27, title: "경로 비용·추종 품질 (그룹/경로별 실행 평균)",
  headers: ["그룹-경로", "참조 거리 (m)", "GT 표본거리 (m)", "추종 RMSE (m)", "추종 P95 (m)", "PX4 heading RMSE (deg)", "GT +X heading RMSE (deg)"],
  rows: runPaths.map(([label]) => [label, null, null, null, null, null, null]),
  formulaColumns: {
    B: runPaths.map(([, group, path]) => averageIfs("02_경로추종", "F", 6, 44, [["B", group], ["C", path]])),
    C: runPaths.map(([, group, path]) => averageIfs("02_경로추종", "G", 6, 44, [["B", group], ["C", path]])),
    D: runPaths.map(([, group, path]) => averageIfs("02_경로추종", "L", 6, 44, [["B", group], ["C", path]])),
    E: runPaths.map(([, group, path]) => averageIfs("02_경로추종", "M", 6, 44, [["B", group], ["C", path]])),
    F: runPaths.map(([, group, path]) => averageIfs("02_경로추종", "Q", 6, 44, [["B", group], ["C", path]])),
    G: runPaths.map(([, group, path]) => averageIfs("02_경로추종", "R", 6, 44, [["B", group], ["C", path]])),
  },
  numberFormats: { B: "0.00", C: "0.00", D: "0.00", E: "0.00", F: "0.000", G: "0.000" },
});

// Observation-quality summaries keep whole-frame and science-pair denominators distinct.
writeTable({
  titleRow: 39, headerRow: 40, title: "카메라·본 구간 검출률 (전체 프레임과 stereo pair 분모 구분)",
  headers: ["그룹-경로", "좌 전체 검출률", "우 전체 검출률", "본 구간 양측 검출률"],
  rows: runPaths.map(([label]) => [label, null, null, null]),
  formulaColumns: {
    B: runPaths.map(([, group, path]) => averageAcrossRunIds("F", group, path)),
    C: runPaths.map(([, group, path]) => averageAcrossRunIds("G", group, path)),
    D: runPaths.map(([, group, path]) => averageAcrossRunIds("J", group, path)),
  },
  numberFormats: { B: "0.0%", C: "0.0%", D: "0.0%" },
});

writeTable({
  titleRow: 52, headerRow: 53, title: "추론 커버리지와 시도 중 검출률 (분모 구분)",
  headers: ["그룹-경로", "좌 추론 시도율", "우 추론 시도율", "좌 시도 중 검출률", "우 시도 중 검출률"],
  rows: runPaths.map(([label]) => [label, null, null, null, null]),
  formulaColumns: {
    B: runPaths.map(([, group, path]) => averageAcrossRunIds("S", group, path)),
    C: runPaths.map(([, group, path]) => averageAcrossRunIds("T", group, path)),
    D: runPaths.map(([, group, path]) => averageAcrossRunIds("Y", group, path)),
    E: runPaths.map(([, group, path]) => averageAcrossRunIds("Z", group, path)),
  },
  numberFormats: { B: "0.0%", C: "0.0%", D: "0.0%", E: "0.0%" },
});

writeTable({
  titleRow: 65, headerRow: 66, title: "실제 카메라·GT 표본 주파수 (Hz)",
  headers: ["그룹-경로", "좌 카메라 Hz", "우 카메라 Hz", "GT Hz"],
  rows: runPaths.map(([label]) => [label, null, null, null]),
  formulaColumns: {
    B: runPaths.map(([, group, path]) => averageAcrossRunIds("D", group, path)),
    C: runPaths.map(([, group, path]) => averageAcrossRunIds("E", group, path)),
    D: runPaths.map(([, group, path]) => averageAcrossRunIds("N", group, path)),
  },
  numberFormats: { B: "0.00", C: "0.00", D: "0.00" },
});

writeTable({
  titleRow: 78, headerRow: 79, title: "pair·GT 기록 무결성 (실행 평균 건수; max pair 차이는 ms)",
  headers: ["그룹-경로", "좌 비매칭 수", "우 비매칭 수", "GT ≥40ms gap 수", "max pair Δt (ms)"],
  rows: runPaths.map(([label]) => [label, null, null, null, null]),
  formulaColumns: {
    B: runPaths.map(([, group, path]) => averageAcrossRunIds("L", group, path)),
    C: runPaths.map(([, group, path]) => averageAcrossRunIds("M", group, path)),
    D: runPaths.map(([, group, path]) => averageAcrossRunIds("O", group, path)),
    E: runPaths.map(([, group, path]) => `=IFERROR(${averageAcrossRunIds("K", group, path).slice(1)}/1000000,"")`),
  },
  numberFormats: { B: "0.0", C: "0.0", D: "0.0", E: "0.00" },
});

// Frozen V0 holdout position errors by calibration route: primary A/B methods and C diagnostic are charted separately to preserve scale.
writeTable({
  titleRow: 91, headerRow: 92, title: "V0 홀드아웃: 경로별 3D RMSE (A/B RPY_IMU, yaw 입력군 평균; 동결 후 기술 비교)",
  headers: ["교정 경로", "A all_data", "B all_data", "A matched_200", "B matched_200"],
  rows: calibrationPaths.map(([label]) => [label, null, null, null, null]),
  formulaColumns: {
    B: calibrationPaths.map(([, group, path]) => averageIfs("04_교정비교", "Y", 6, 819, [["A", "EXISTING_YAW"], ["B", group], ["C", path], ["E", "all_data"], ["F", "A_rtk_independent"], ["G", "RPY_IMU"]])),
    C: calibrationPaths.map(([, group, path]) => averageIfs("04_교정비교", "Y", 6, 819, [["A", "EXISTING_YAW"], ["B", group], ["C", path], ["E", "all_data"], ["F", "B_rtk_joint"], ["G", "RPY_IMU"]])),
    D: calibrationPaths.map(([, group, path]) => averageIfs("04_교정비교", "Y", 6, 819, [["A", "EXISTING_YAW"], ["B", group], ["C", path], ["E", "matched_200"], ["F", "A_rtk_independent"], ["G", "RPY_IMU"]])),
    E: calibrationPaths.map(([, group, path]) => averageIfs("04_교정비교", "Y", 6, 819, [["A", "EXISTING_YAW"], ["B", group], ["C", path], ["E", "matched_200"], ["F", "B_rtk_joint"], ["G", "RPY_IMU"]])),
  },
  numberFormats: { B: "0.000", C: "0.000", D: "0.000", E: "0.000" },
});

writeTable({
  titleRow: 104, headerRow: 105, title: "C_no_drone_rtk_joint 진단: 경로별 3D RMSE (별도 축)",
  headers: ["교정 경로", "C all_data RMSE (m)", "C matched_200 RMSE (m)", "log10 all_data RMSE", "log10 matched_200 RMSE"],
  rows: calibrationPaths.map(([label]) => [label, null, null, null, null]),
  formulaColumns: {
    B: calibrationPaths.map(([, group, path]) => averageIfs("04_교정비교", "Y", 6, 819, [["A", "EXISTING_YAW"], ["B", group], ["C", path], ["E", "all_data"], ["F", "C_no_drone_rtk_joint"], ["G", "RPY_IMU"]])),
    C: calibrationPaths.map(([, group, path]) => averageIfs("04_교정비교", "Y", 6, 819, [["A", "EXISTING_YAW"], ["B", group], ["C", path], ["E", "matched_200"], ["F", "C_no_drone_rtk_joint"], ["G", "RPY_IMU"]])),
    D: calibrationPaths.map((_, i) => `=IFERROR(LOG10(B${106 + i}),"")`),
    E: calibrationPaths.map((_, i) => `=IFERROR(LOG10(C${106 + i}),"")`),
  },
  numberFormats: { B: "0.00", C: "0.00", D: "0.00", E: "0.00" },
});

const p2MainCases = [
  ["A · RPY_IMU", "A_rtk_independent", "RPY_IMU"],
  ["A · YAW_ONLY", "A_rtk_independent", "YAW_ONLY"],
  ["B · RPY_IMU", "B_rtk_joint", "RPY_IMU"],
  ["B · YAW_ONLY", "B_rtk_joint", "YAW_ONLY"],
];
writeTable({
  titleRow: 117, headerRow: 118, title: "선정 경로 P2: 3D RMSE/P95 (A/B; existing-yaw 조건 평균)",
  headers: ["method / attitude", "RMSE all_data (m)", "P95 all_data (m)", "RMSE matched_200 (m)", "P95 matched_200 (m)"],
  rows: p2MainCases.map(([label]) => [label, null, null, null, null]),
  formulaColumns: {
    B: p2MainCases.map(([, method, variant]) => averageIfs("04_교정비교", "Y", 6, 819, [["A", "EXISTING_YAW"], ["B", "G0"], ["C", "P2"], ["E", "all_data"], ["F", method], ["G", variant]])),
    C: p2MainCases.map(([, method, variant]) => averageIfs("04_교정비교", "Z", 6, 819, [["A", "EXISTING_YAW"], ["B", "G0"], ["C", "P2"], ["E", "all_data"], ["F", method], ["G", variant]])),
    D: p2MainCases.map(([, method, variant]) => averageIfs("04_교정비교", "Y", 6, 819, [["A", "EXISTING_YAW"], ["B", "G0"], ["C", "P2"], ["E", "matched_200"], ["F", method], ["G", variant]])),
    E: p2MainCases.map(([, method, variant]) => averageIfs("04_교정비교", "Z", 6, 819, [["A", "EXISTING_YAW"], ["B", "G0"], ["C", "P2"], ["E", "matched_200"], ["F", method], ["G", variant]])),
  },
  numberFormats: { B: "0.000", C: "0.000", D: "0.000", E: "0.000" },
});

const p2Cdiagnostic = [
  ["C · RPY_IMU", "RPY_IMU"],
  ["C · YAW_ONLY", "YAW_ONLY"],
];
writeTable({
  titleRow: 130, headerRow: 131, title: "P2 C_no_drone_rtk_joint 진단: RMSE/P95 (별도 축)",
  headers: ["attitude", "RMSE all_data (m)", "P95 all_data (m)", "RMSE matched_200 (m)", "P95 matched_200 (m)"],
  rows: p2Cdiagnostic.map(([label]) => [label, null, null, null, null]),
  formulaColumns: {
    B: p2Cdiagnostic.map(([, variant]) => averageIfs("04_교정비교", "Y", 6, 819, [["A", "EXISTING_YAW"], ["B", "G0"], ["C", "P2"], ["E", "all_data"], ["F", "C_no_drone_rtk_joint"], ["G", variant]])),
    C: p2Cdiagnostic.map(([, variant]) => averageIfs("04_교정비교", "Z", 6, 819, [["A", "EXISTING_YAW"], ["B", "G0"], ["C", "P2"], ["E", "all_data"], ["F", "C_no_drone_rtk_joint"], ["G", variant]])),
    D: p2Cdiagnostic.map(([, variant]) => averageIfs("04_교정비교", "Y", 6, 819, [["A", "EXISTING_YAW"], ["B", "G0"], ["C", "P2"], ["E", "matched_200"], ["F", "C_no_drone_rtk_joint"], ["G", variant]])),
    E: p2Cdiagnostic.map(([, variant]) => averageIfs("04_교정비교", "Z", 6, 819, [["A", "EXISTING_YAW"], ["B", "G0"], ["C", "P2"], ["E", "matched_200"], ["F", "C_no_drone_rtk_joint"], ["G", variant]])),
  },
  numberFormats: { B: "0.00", C: "0.00", D: "0.00", E: "0.00" },
});

const axisRows = [];
const axisRMSEColumns = { lateral: "AC", forward: "AD", vertical: "AE" };
const axisBiasColumns = { lateral: "AF", forward: "AG", vertical: "AH" };
for (const [method, short] of mainMethods.map(m => [m, m.startsWith("A_") ? "A" : "B"])) {
  for (const [axis, rmseCol, biasCol] of [["횡", "AC", "AF"], ["전방", "AD", "AG"], ["수직", "AE", "AH"]]) {
    axisRows.push({ label: `${short} · ${axis}`, method, rmseCol, biasCol });
  }
}
writeTable({
  titleRow: 143, headerRow: 144, title: "선정 경로 P2: 축별 오차 (A/B, all_data, RPY_IMU; yaw-case 평균)",
  headers: ["method · 축", "RMSE (m)", "bias (m)"],
  rows: axisRows.map(r => [r.label, null, null]),
  formulaColumns: {
    B: axisRows.map(r => averageIfs("04_교정비교", r.rmseCol, 6, 819, [["A", "EXISTING_YAW"], ["B", "G0"], ["C", "P2"], ["E", "all_data"], ["F", r.method], ["G", "RPY_IMU"]])),
    C: axisRows.map(r => averageIfs("04_교정비교", r.biasCol, 6, 819, [["A", "EXISTING_YAW"], ["B", "G0"], ["C", "P2"], ["E", "all_data"], ["F", r.method], ["G", "RPY_IMU"]])),
  },
  numberFormats: { B: "0.000", C: "0.000" },
});

const orientationMethods = ["A_rtk_independent", "B_rtk_joint"];
writeTable({
  titleRow: 156, headerRow: 157, title: "선정 경로 P2: 자세 오차 (A/B, all_data, RPY_IMU; deg)",
  headers: ["method", "좌 회전오차", "우 회전오차", "상대 yaw 절대오차"],
  rows: orientationMethods.map(m => [m, null, null, null]),
  formulaColumns: {
    B: orientationMethods.map(m => averageCaseCells([["A", "EXISTING_YAW"], ["B", "G0"], ["C", "P2"], ["E", "all_data"], ["F", m], ["G", "RPY_IMU"]], "W")),
    C: orientationMethods.map(m => averageCaseCells([["A", "EXISTING_YAW"], ["B", "G0"], ["C", "P2"], ["E", "all_data"], ["F", m], ["G", "RPY_IMU"]], "X")),
    D: orientationMethods.map(m => averageCaseCells([["A", "EXISTING_YAW"], ["B", "G0"], ["C", "P2"], ["E", "all_data"], ["F", m], ["G", "RPY_IMU"]], "V")),
  },
  numberFormats: { B: "0.000", C: "0.000", D: "0.000" },
});

const availabilityCases = [
  ["A · RPY", "A_rtk_independent", "RPY_IMU"],
  ["A · YAW", "A_rtk_independent", "YAW_ONLY"],
  ["B · RPY", "B_rtk_joint", "RPY_IMU"],
  ["B · YAW", "B_rtk_joint", "YAW_ONLY"],
  ["C · RPY", "C_no_drone_rtk_joint", "RPY_IMU"],
  ["C · YAW", "C_no_drone_rtk_joint", "YAW_ONLY"],
];
const baseCriteria = ([, method, variant]) => [["A", "EXISTING_YAW"], ["B", "G0"], ["C", "P2"], ["E", "all_data"], ["F", method], ["G", variant]];
writeTable({
  titleRow: 169, headerRow: 170, title: "선정 경로 P2: 수렴·위치지표 가용성 (all_data)",
  headers: ["method · attitude", "수렴률", "위치지표 가용률", "유효 측정률", "검출률"],
  rows: availabilityCases.map(([label]) => [label, null, null, null, null]),
  formulaColumns: {
    B: availabilityCases.map(item => sumRatio("04_교정비교", "K", "H", 6, 819, baseCriteria(item))),
    C: availabilityCases.map(item => sumRatio("04_교정비교", "M", "L", 6, 819, baseCriteria(item))),
    D: availabilityCases.map(([, method, variant]) => averageIfs("04_교정비교", "AA", 6, 819, [["A", "EXISTING_YAW"], ["B", "G0"], ["C", "P2"], ["E", "all_data"], ["F", method], ["G", variant]])),
    E: availabilityCases.map(([, method, variant]) => averageIfs("04_교정비교", "AB", 6, 819, [["A", "EXISTING_YAW"], ["B", "G0"], ["C", "P2"], ["E", "all_data"], ["F", method], ["G", variant]])),
  },
  numberFormats: { B: "0.0%", C: "0.0%", D: "0.0%", E: "0.0%" },
});

writeTable({
  titleRow: 182, headerRow: 183, title: "RPY_IMU 대 YAW_ONLY: 비교 가능 사례의 개선·악화 비율",
  headers: ["method", "개선률 (비교 가능 사례)", "악화률 (비교 가능 사례)", "비교 불가 건수"],
  rows: [
    ["A_rtk_independent", null, null, null],
    ["B_rtk_joint", null, null, null],
    ["C_no_drone_rtk_joint", null, null, null],
  ],
  formulaColumns: {
    B: [21, 22, 23].map(r => `=IFERROR('05_그래프'!B${r}/('05_그래프'!B${r}+'05_그래프'!C${r}+'05_그래프'!D${r}),"")`),
    C: [21, 22, 23].map(r => `=IFERROR('05_그래프'!C${r}/('05_그래프'!B${r}+'05_그래프'!C${r}+'05_그래프'!D${r}),"")`),
    D: [21, 22, 23].map(r => `='05_그래프'!E${r}`),
  },
  numberFormats: { B: "0.0%", C: "0.0%", D: "0" },
});

const scatterPaths = [
  ["G0-P0", "G0", "P0", 6], ["G0-P1", "G0", "P1", 7], ["G0-P2", "G0", "P2", 8],
  ["G0-P3", "G0", "P3", 9], ["G0-P4", "G0", "P4", 10],
];
writeTable({
  titleRow: 195, headerRow: 196, title: "Trade-off: G0 교정 경로 실제 시간과 V0 holdout 3D RMSE (all_data, RPY_IMU)",
  headers: ["경로", "실제 구간 시간 (s)", "A RMSE (m)", "B RMSE (m)", "C RMSE (m)"],
  rows: scatterPaths.map(([label]) => [label, null, null, null, null]),
  formulaColumns: {
    B: scatterPaths.map(([, , , row]) => `='05_그래프'!C${row}`),
    C: scatterPaths.map(([, group, path]) => averageIfs("04_교정비교", "Y", 6, 819, [["A", "EXISTING_YAW"], ["B", group], ["C", path], ["E", "all_data"], ["F", "A_rtk_independent"], ["G", "RPY_IMU"]])),
    D: scatterPaths.map(([, group, path]) => averageIfs("04_교정비교", "Y", 6, 819, [["A", "EXISTING_YAW"], ["B", group], ["C", path], ["E", "all_data"], ["F", "B_rtk_joint"], ["G", "RPY_IMU"]])),
    E: scatterPaths.map(([, group, path]) => averageIfs("04_교정비교", "Y", 6, 819, [["A", "EXISTING_YAW"], ["B", group], ["C", path], ["E", "all_data"], ["F", "C_no_drone_rtk_joint"], ["G", "RPY_IMU"]])),
  },
  numberFormats: { B: "0.000", C: "0.000", D: "0.000", E: "0.00" },
});

// A/B method timing scatter uses a shared numeric scale; C is shown in its own diagnostic chart.
writeTable({
  titleRow: 208, headerRow: 209, title: "C_no_drone_rtk_joint: 시간–오차 scatter (log10 별도 축)",
  headers: ["경로", "실제 구간 시간 (s)", "log10(RMSE / m)"],
  rows: scatterPaths.map(([label]) => [label, null, null]),
  formulaColumns: {
    B: scatterPaths.map(([, , , row]) => `='05_그래프'!C${row}`),
    C: scatterPaths.map(([, group, path], i) => `=IFERROR(LOG10(${averageIfs("04_교정비교", "Y", 6, 819, [["A", "EXISTING_YAW"], ["B", group], ["C", path], ["E", "all_data"], ["F", "C_no_drone_rtk_joint"], ["G", "RPY_IMU"]]).slice(1)}),"")`),
  },
  numberFormats: { B: "0.000", C: "0.0" },
});

const p2GpsCases = [
  ["A · RPY_IMU", "A_rtk_independent", "RPY_IMU"],
  ["A · YAW_ONLY", "A_rtk_independent", "YAW_ONLY"],
  ["B · RPY_IMU", "B_rtk_joint", "RPY_IMU"],
  ["B · YAW_ONLY", "B_rtk_joint", "YAW_ONLY"],
];
writeTable({
  titleRow: 217, headerRow: 218, title: "P2 GPS_YAW90 diagnostic: A/B 3D RMSE/P95 by attitude and scope",
  headers: ["method / attitude", "RMSE all_data (m)", "P95 all_data (m)", "RMSE matched_200 (m)", "P95 matched_200 (m)"],
  rows: p2GpsCases.map(([label]) => [label, null, null, null, null]),
  formulaColumns: {
    B: p2GpsCases.map(([, method, variant]) => averageCaseCells([["A", "GPS_YAW90"], ["B", "G0"], ["C", "P2"], ["E", "all_data"], ["F", method], ["G", variant]], "Y")),
    C: p2GpsCases.map(([, method, variant]) => averageCaseCells([["A", "GPS_YAW90"], ["B", "G0"], ["C", "P2"], ["E", "all_data"], ["F", method], ["G", variant]], "Z")),
    D: p2GpsCases.map(([, method, variant]) => averageCaseCells([["A", "GPS_YAW90"], ["B", "G0"], ["C", "P2"], ["E", "matched_200"], ["F", method], ["G", variant]], "Y")),
    E: p2GpsCases.map(([, method, variant]) => averageCaseCells([["A", "GPS_YAW90"], ["B", "G0"], ["C", "P2"], ["E", "matched_200"], ["F", method], ["G", variant]], "Z")),
  },
  numberFormats: { B: "0.000", C: "0.000", D: "0.000", E: "0.000" },
});

// Chart additions: do not source any run-attempt status or failure-reason fields.
const colors = ["#4E79A7", "#F28E2B", "#59A14F", "#9C755F", "#76B7B2", "#B279A2"];
const added = [];
function recordChart(chart, title, type = "bar") { added.push({ title, type, series: chart.series.items.length }); }

let chart = addChart("bar", [graph.getRange("A27:A36"), graph.getRange("B27:B36"), graph.getRange("C27:C36")],
  "경로 거리: 참조 vs GT 표본거리 (m)", "0.0", "거리 (m)", { xTitle: "그룹-경로", colors: [colors[0], colors[1]] });
recordChart(chart, "경로 거리: 참조 vs GT 표본거리 (m)");

chart = addChart("bar", [graph.getRange("A27:A36"), graph.getRange("D27:D36"), graph.getRange("E27:E36")],
  "경로 추종: 3D RMSE vs P95 (m, 실행 평균)", "0.0", "추종 오차 (m)", { xTitle: "그룹-경로", colors: [colors[0], colors[1]] });
recordChart(chart, "경로 추종 RMSE/P95");

chart = addChart("bar", [graph.getRange("A27:A36"), graph.getRange("F27:F36"), graph.getRange("G27:G36")],
  "경로 heading 오차: PX4 vs GT +X (deg, 실행 평균)", "0.00", "heading RMSE (deg)", { xTitle: "그룹-경로", colors: [colors[0], colors[1]] });
recordChart(chart, "heading RMSE");

chart = addChart("bar", graph.getRange("A40:D49"),
  "영상 검출률: 좌/우 전체 프레임과 본 구간 양측 pair (%)", "0%", "검출률", { xTitle: "그룹-경로", colors: [colors[0], colors[1], colors[2]] });
recordChart(chart, "전체/본 구간 검출률");

chart = addChart("bar", graph.getRange("A53:E62"),
  "추론 커버리지 vs 시도 중 검출률 (%)", "0%", "비율", { xTitle: "그룹-경로", colors: [colors[0], colors[1], colors[2], colors[3]] });
recordChart(chart, "추론 커버리지/조건부 검출률");

chart = addChart("bar", graph.getRange("A66:D75"),
  "실제 camera/GT sampling frequency (Hz)", "0.0", "실제 표본 주파수 (Hz)", { xTitle: "그룹-경로", colors: [colors[0], colors[1], colors[2]] });
recordChart(chart, "실제 camera/GT sampling frequency");

chart = addChart("bar", graph.getRange("A79:D88"),
  "기록 무결성: 비매칭 프레임과 GT gap (실행 평균 건수)", "0.0", "건수", { xTitle: "그룹-경로", colors: [colors[0], colors[1], colors[2]] });
recordChart(chart, "비매칭 프레임/GT gap");

chart = addChart("bar", graph.getRange("A92:E99"),
  "V0 holdout 3D RMSE by calibration path (A/B, RPY_IMU; all_data vs matched_200)", "0.00", "3D RMSE (m)", { xTitle: "교정 경로 · 결과는 동결 후 기술 비교", colors: [colors[0], colors[1], colors[2], colors[3]] });
recordChart(chart, "route RMSE A/B, all vs matched");

chart = addChart("bar", [graph.getRange("A105:A112"), graph.getRange("D105:D112"), graph.getRange("E105:E112")],
  "C_no_drone_rtk_joint diagnostic: route 3D RMSE (log10 scale)", "0.0", "log10(RMSE / m)", { xTitle: "교정 경로 · V0 holdout", colors: [colors[4], colors[5]] });
recordChart(chart, "route RMSE C diagnostic");

chart = addChart("bar", graph.getRange("A118:E122"),
  "선정 P2: A/B 3D RMSE/P95 by method × attitude × sample scope (existing-yaw mean)", "0.00", "3D position error (m)", { xTitle: "method · attitude", colors: [colors[0], colors[1], colors[2], colors[3]] });
recordChart(chart, "P2 RMSE/P95 A/B");

chart = addChart("bar", graph.getRange("A131:E133"),
  "P2 C_no_drone_rtk_joint diagnostic: RMSE/P95 by attitude/scope (별도 축)", "0.0", "3D position error (m)", { xTitle: "attitude", colors: [colors[4], colors[5], colors[0], colors[1]] });
recordChart(chart, "P2 RMSE/P95 C diagnostic");

chart = addChart("bar", graph.getRange("A144:C150"),
  "선정 P2: 축별 RMSE와 signed bias (A/B, all_data, RPY_IMU, m)", "0.00", "error (m)", { xTitle: "method · 축", colors: [colors[0], colors[1]] });
recordChart(chart, "P2 axis RMSE/bias A/B");

chart = addChart("bar", graph.getRange("A157:D159"),
  "선정 P2: 카메라 자세 오차 (A/B, all_data, RPY_IMU, deg)", "0.000", "angular error (deg)", { xTitle: "method", colors: [colors[0], colors[1], colors[2]] });
recordChart(chart, "P2 camera attitude errors");

chart = addChart("bar", graph.getRange("A170:E176"),
  "선정 P2: 수렴·위치지표 가용성 (all_data, existing-yaw mean, %)", "0.0%", "rate", { xTitle: "method · attitude", colors: [colors[0], colors[1], colors[2], colors[3]], minY: 0, maxY: 1 });
recordChart(chart, "P2 convergence/availability");

chart = addChart("bar", graph.getRange("A183:C186"),
  "RPY_IMU vs YAW_ONLY: 개선/악화 비율 (비교 가능 사례만, %)", "0%", "share of comparable pairs", { xTitle: "method", colors: [colors[2], colors[1]] });
recordChart(chart, "paired improvement/worsening share");

chart = addChart("scatter", graph.getRange("B196:D201"),
  "Trade-off: 실제 비행시간 vs V0 3D RMSE (A/B RPY_IMU, all_data; G0 P0–P4)", "0.00", "3D RMSE (m)", { xTitle: "실제 본 구간 시간 (s)", colors: [colors[0], colors[1]] });
recordChart(chart, "accuracy-time scatter A/B", "scatter");
applyScatterPointLabels(chart, scatterPaths.map(([label]) => label), [0]);

chart = addChart("scatter", graph.getRange("B209:C214"),
  "Trade-off diagnostic: C_no_drone_rtk_joint (log10 y축; G0 P0–P4)", "0.0", "log10(RMSE / m)", { xTitle: "실제 본 구간 시간 (s)", colors: [colors[4]], legend: false });
recordChart(chart, "accuracy-time scatter C", "scatter");
applyScatterPointLabels(chart, scatterPaths.map(([label]) => label));

chart = addChart("bar", graph.getRange("A218:E222"),
  "P2 GPS_YAW90 diagnostic: A/B 3D RMSE/P95 by attitude/scope", "0.00", "3D position error (m)", { xTitle: "method · attitude", colors: [colors[0], colors[1], colors[2], colors[3]] });
recordChart(chart, "P2 GPS_YAW90 diagnostic");

workbook.recalculate();

const helperRange = graph.getRange("A26:G222");
const helperValues = helperRange.values;
const formulaErrors = [];
for (let r = 0; r < helperValues.length; r++) {
  for (let c = 0; c < helperValues[r].length; c++) {
    const value = helperValues[r][c];
    if (typeof value === "string" && /^#(REF|DIV\/0|VALUE|NAME|N\/A|NUM|NULL)/.test(value)) {
      formulaErrors.push({ row: r + 26, col: c + 1, value });
    }
  }
}
if (formulaErrors.length) throw new Error(`Formula errors in graph data: ${JSON.stringify(formulaErrors.slice(0, 20))}`);

const routeAccuracy = graph.getRange("A92:E99").values;
const p2Accuracy = graph.getRange("A118:E122").values;
const checkValueCells = [...routeAccuracy.flat(), ...p2Accuracy.flat()];
if (!checkValueCells.some(v => typeof v === "number" && Number.isFinite(v))) {
  throw new Error("Calibration graph summaries did not calculate numeric values.");
}

function assertNumericRange(address) {
  const values = graph.getRange(address).values.flat();
  const missingIndexes = values.flatMap((value, index) =>
    typeof value === "number" && Number.isFinite(value) ? [] : [index],
  );
  if (missingIndexes.length) {
    throw new Error(`Expected numeric chart data in ${address}; missing/non-numeric cells at offsets ${missingIndexes.join(",")}`);
  }
}
for (const address of [
  "B28:G36", "B41:D49", "B54:E62", "B67:D75", "B80:E88",
  "B93:E99", "B106:E112", "B119:E122", "B132:E133", "B145:C150",
  "B158:D159", "B171:E176", "B184:D186", "B197:E201", "B210:C214",
  "B219:E222",
]) assertNumericRange(address);

const xlsx = await SpreadsheetFile.exportXlsx(workbook);
await xlsx.save(candidatePath);

// Re-import the candidate and verify the edited graph plus untouched tabs.
const roundtrip = await SpreadsheetFile.importXlsx(await FileBlob.load(candidatePath));
if (JSON.stringify(roundtrip.worksheets.items.map(s => s.name)) !== JSON.stringify(worksheetNamesBefore)) {
  throw new Error("Worksheet names/order changed during export.");
}
for (const [name, digest] of nonGraphBefore) {
  const sheet = roundtrip.worksheets.getItem(name);
  if (hashJson(snapshotSheet(sheet)) !== digest) throw new Error(`Unrelated sheet changed: ${name}`);
}
const graphAfter = roundtrip.worksheets.getItem("05_그래프");
if (JSON.stringify(graphAfter.getRange("A5:F14").values) !== JSON.stringify(originalGraphTable1)) throw new Error("Existing path cost table changed.");
if (JSON.stringify(graphAfter.getRange("A20:E23").values) !== JSON.stringify(originalGraphTable2)) throw new Error("Existing paired comparison table changed.");
if (JSON.stringify(tableSignature(graphAfter)) !== JSON.stringify(originalGraphTables)) throw new Error("Existing native tables changed.");
const afterCharts = chartSignature(graphAfter);
if (afterCharts.length !== originalChartSignature.length + added.length) throw new Error(`Unexpected chart count after round-trip: ${afterCharts.length}`);
for (let i = 0; i < originalChartSignature.length; i++) {
  if (JSON.stringify(afterCharts[i]) !== JSON.stringify(originalChartSignature[i])) throw new Error(`Existing chart ${i + 1} changed unexpectedly.`);
}
const chartSources = afterCharts.slice(originalChartSignature.length).flatMap(c => c.series.map(s => `${s.formula} ${s.categoryFormula}`)).join("\n");
if (chartSources.includes("01_실행현황")) throw new Error("An execution-status source was included in a new graph.");

await fs.mkdir(previewDir, { recursive: true });
const previewRanges = [
  ["top", "A1:Q95"],
  ["flight_observation", "A96:Q190"],
  ["calibration", "A191:Q290"],
  ["methods_tradeoff", "A291:Q410"],
];
for (const [name, range] of previewRanges) {
  const preview = await roundtrip.render({ sheetName: "05_그래프", range, scale: 1, format: "png" });
  await fs.writeFile(`${previewDir}/${name}.png`, new Uint8Array(await preview.arrayBuffer()));
}

console.log(JSON.stringify({
  status: "CANDIDATE_BUILT_AND_VERIFIED",
  candidatePath,
  existingChartCount: originalChartSignature.length,
  addedChartCount: added.length,
  totalChartCount: afterCharts.length,
  addedCharts: added,
  helperFormulaErrors: formulaErrors.length,
  routeAccuracySample: routeAccuracy.slice(0, 3),
  p2AccuracySample: p2Accuracy.slice(0, 3),
  unchangedNonGraphSheets: [...nonGraphBefore.keys()],
  graphNativeTablesPreserved: graphAfter.tables.items.length,
  chartSourcesExcludeExecutionStatus: true,
  previewPaths: previewRanges.map(([name]) => `${previewDir}/${name}.png`),
}, null, 2));
