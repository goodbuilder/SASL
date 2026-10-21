import fs from "node:fs/promises";
import { createReadStream } from "node:fs";
import { createHash } from "node:crypto";
import path from "node:path";
import readline from "node:readline";
import { FileBlob, SpreadsheetFile } from "@oai/artifact-tool";

async function findRepoRoot(start) {
  let current = start;
  while (true) {
    try {
      await fs.access(path.join(current, ".git"));
      return current;
    } catch {
    }
    const parent = path.dirname(current);
    if (parent === current) throw new Error("Could not locate repository root.");
    current = parent;
  }
}

const repo = await findRepoRoot(process.cwd());
const relativeCampaign = "apps/Simulation/recordings/stereo_v1_1/paper1_v1_2_001";
const campaign = path.join(repo, relativeCampaign);
const analysisRel = relativeCampaign + "/analysis_versions/analysis_9cfd7c6cd10b";
const analysis = path.join(repo, analysisRel);
const analysisId = "analysis_9cfd7c6cd10b";
const sourceWorkbookRel = relativeCampaign + "/stereo_v1_2_summary.xlsx";
const sourceWorkbookPath = path.join(repo, sourceWorkbookRel);
const outputDirRel = relativeCampaign + "/outputs/20261021-postfreeze-summary-review";
const outputDir = path.join(repo, outputDirRel);
const outputPath = path.join(outputDir, "stereo_v1_2_summary_postfreeze_reviewed.xlsx");
// Bounded disposable renders only; they are regenerated under /tmp and are not analysis inputs.
const previewDir = "/tmp/sasl-postfreeze-review/previews";
const font = "Noto Sans CJK KR";
const navy = "#253B56";
const ink = "#172033";
const band = "#EDF3F8";
const teal = "#DDF1EC";
const amber = "#FFF0C2";

async function sha256(filePath) {
  const hash = createHash("sha256");
  for await (const chunk of createReadStream(filePath)) hash.update(chunk);
  return hash.digest("hex");
}

async function* lines(filePath) {
  const input = createReadStream(filePath, { encoding: "utf8" });
  const reader = readline.createInterface({ input, crlfDelay: Infinity });
  for await (const line of reader) yield line;
}

function parseCsvLine(line) {
  const fields = [];
  let value = "";
  let quoted = false;
  for (let i = 0; i < line.length; i += 1) {
    const ch = line[i];
    if (quoted) {
      if (ch === '"' && line[i + 1] === '"') {
        value += '"';
        i += 1;
      } else if (ch === '"') {
        quoted = false;
      } else {
        value += ch;
      }
    } else if (ch === '"') {
      quoted = true;
    } else if (ch === ",") {
      fields.push(value);
      value = "";
    } else {
      value += ch;
    }
  }
  fields.push(value);
  return fields;
}

async function* csvRows(filePath) {
  let header;
  for await (const line of lines(filePath)) {
    if (header === undefined) {
      header = parseCsvLine(line);
      continue;
    }
    if (line.length === 0) continue;
    const values = parseCsvLine(line);
    if (values.length !== header.length) {
      throw new Error("CSV field count mismatch in " + filePath);
    }
    const row = {};
    for (let i = 0; i < header.length; i += 1) row[header[i]] = values[i];
    yield row;
  }
}

function numberOrNull(value) {
  if (value === "" || value === null || value === undefined) return null;
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : null;
}

function quantile(values, probability) {
  if (!values.length) return null;
  const sorted = [...values].sort((a, b) => a - b);
  const position = (sorted.length - 1) * probability;
  const lower = Math.floor(position);
  const upper = Math.ceil(position);
  if (lower === upper) return sorted[lower];
  const weight = position - lower;
  return sorted[lower] * (1 - weight) + sorted[upper] * weight;
}

function stats(values) {
  const finite = values.filter(Number.isFinite);
  if (!finite.length) {
    return { n: 0, mean: null, sd: null, median: null, q025: null, q975: null };
  }
  const mean = finite.reduce((sum, value) => sum + value, 0) / finite.length;
  const variance = finite.length > 1
    ? finite.reduce((sum, value) => sum + (value - mean) ** 2, 0) / (finite.length - 1)
    : null;
  return {
    n: finite.length,
    mean,
    sd: variance === null ? null : Math.sqrt(variance),
    median: quantile(finite, 0.5),
    q025: quantile(finite, 0.025),
    q975: quantile(finite, 0.975),
  };
}

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

if (process.argv.includes("--fix-formula-map-only")) {
  const workbook = await SpreadsheetFile.importXlsx(await FileBlob.load(outputPath));
  const formulaMap = workbook.worksheets.getItem("10_계산추적");
  formulaMap.getRange("F5:F6").values = [
    ["'='02_경로추종'!M<원본행>"],
    ["'=IF(Frow>1,\"P95 경고\",\"기준 이내\")"],
  ];
  workbook.recalculate();
  const examples = formulaMap.getRange("F5:F6");
  const exampleValues = examples.values.map((row) => row[0]);
  const exampleFormulas = examples.formulas.map((row) => row[0]);
  assert(exampleValues.every((value) => typeof value === "string" && value.startsWith("=")),
    "Formula Map examples were not retained as literal text.");
  assert(exampleFormulas.every((formula) => formula === "" || formula === null),
    "Formula Map examples were interpreted as live formulas.");
  const fixedFile = await SpreadsheetFile.exportXlsx(workbook);
  await fixedFile.save(outputPath);
  const reopened = await SpreadsheetFile.importXlsx(await FileBlob.load(outputPath));
  const reopenedExamples = reopened.worksheets.getItem("10_계산추적").getRange("F5:F6");
  assert(reopenedExamples.values.every((row) => typeof row[0] === "string" && row[0].startsWith("=")),
    "Literal formula examples did not survive XLSX round-trip.");
  assert(reopenedExamples.formulas.every((row) => row[0] === "" || row[0] === null),
    "Formula Map examples became formulas after round-trip.");
  const drawings = chartRecords(await reopened.inspect({ kind: "drawing", sheetId: "05_그래프", maxChars: 120000 }))
    .filter((record) => record.drawingType === "chart");
  assert(drawings.length === 22, "Chart count changed while fixing Formula Map text.");
  const preview = await reopened.render({ sheetName: "10_계산추적", range: "A1:G13", scale: 1, format: "png" });
  await fs.mkdir(previewDir, { recursive: true });
  await fs.writeFile(path.join(previewDir, "formula_trace.png"), new Uint8Array(await preview.arrayBuffer()));
  console.log("FORMULA_MAP_LITERAL_EXAMPLES=" + JSON.stringify(reopenedExamples.values));
  console.log("OUTPUT_SHA256_AFTER_FIX=" + await sha256(outputPath));
  console.log("OUTPUT_PRESENT=" + String((await fs.stat(outputPath)).size));
  process.exit(0);
}

function closeEnough(actual, expected, tolerance = 1e-9) {
  return Number.isFinite(actual) && Number.isFinite(expected)
    && Math.abs(actual - expected) <= tolerance;
}

function parseJsonLines(text) {
  return text.split(/\r?\n/).filter(Boolean).map((line) => JSON.parse(line));
}

function parseRun(runId) {
  const match = /^([A-Z]\d+)-(P\d+|V0)-(R\d+)$/.exec(runId);
  if (!match) throw new Error("Unexpected run ID: " + runId);
  return { rig: match[1], path: match[2], repetition: match[3] };
}

async function expectedAnalysisHashes() {
  const wanted = new Set([
    "analysis_freeze.json",
    "analysis_methodology.md",
    "analysis_run_manifest.json",
    "analysis_summary.json",
    "attitude_variant_comparisons.csv",
    "calibration_metrics.csv",
    "evaluation_metrics.csv",
    "gps_yaw90_calibration_metrics.csv",
    "gps_yaw90_evaluation_metrics.csv",
    "path_selection.json",
  ]);
  const found = new Map();
  for await (const line of lines(path.join(analysis, "analysis_checksums.sha256"))) {
    const parts = line.split("  ", 2);
    if (parts.length !== 2) continue;
    const name = path.basename(parts[1]);
    if (wanted.has(name) && !parts[1].includes("/")) found.set(name, parts[0]);
    if (found.size === wanted.size) break;
  }
  assert(found.size === wanted.size, "Not every requested analysis source has a manifest hash.");
  return found;
}

function checksumEntries(text) {
  const result = new Map();
  for (const line of text.split(/\r?\n/)) {
    const parts = line.split("  ", 2);
    if (parts.length === 2) result.set(parts[1], parts[0]);
  }
  return result;
}

function setColumnWidths(sheet, widths) {
  for (const [column, width] of Object.entries(widths)) {
    sheet.getRange(column + ":" + column).format.columnWidth = width;
  }
}

function styleHeader(sheet, range) {
  sheet.getRange(range).format = {
    fill: navy,
    font: { name: font, size: 9, bold: true, color: "#FFFFFF" },
    horizontalAlignment: "center",
    verticalAlignment: "center",
    wrapText: true,
  };
  sheet.getRange(range).format.rowHeight = 34;
}

function styleBody(sheet, range, rowHeight = 20) {
  sheet.getRange(range).format = {
    font: { name: font, size: 9, color: ink },
    verticalAlignment: "center",
    wrapText: false,
  };
  sheet.getRange(range).format.rowHeight = rowHeight;
}

function addFilterTable(sheet, range, name) {
  const table = sheet.tables.add(range, true, name);
  table.style = "TableStyleMedium2";
  table.showFilterButton = true;
  return table;
}

function snapshotSheet(sheet) {
  const used = sheet.getUsedRange();
  return {
    address: used.address,
    values: used.values,
    formulas: used.formulas,
  };
}

function sameCellValue(left, right) {
  const leftBlank = left === null || left === undefined || left === "";
  const rightBlank = right === null || right === undefined || right === "";
  if (leftBlank && rightBlank) return true;
  return JSON.stringify(left) === JSON.stringify(right);
}

function excelColumnName(zeroBasedIndex) {
  let value = zeroBasedIndex + 1;
  let result = "";
  while (value > 0) {
    const remainder = (value - 1) % 26;
    result = String.fromCharCode(65 + remainder) + result;
    value = Math.floor((value - 1) / 26);
  }
  return result;
}

function excelColumnIndex(columnName) {
  let result = 0;
  for (const letter of columnName) result = result * 26 + letter.charCodeAt(0) - 64;
  return result - 1;
}

function rangeStartCell(address) {
  const match = /^([A-Z]+)(\d+):[A-Z]+(\d+)$/.exec(address);
  if (!match) throw new Error("Unsupported used-range address: " + address);
  return { row: Number(match[2]), column: excelColumnIndex(match[1]) };
}

function chartRecords(inspectResult) {
  return inspectResult.ndjson.split("\n").filter(Boolean).map((line) => JSON.parse(line));
}

function chartDataSignature(workbook, records) {
  return records.map((record) => {
    const chart = workbook.resolve(record.id);
    return {
      type: chart.type,
      series: chart.series.items.map((series) => ({
        formula: series.formula,
        categoryFormula: series.categoryFormula,
      })),
    };
  });
}

const selectedInputNames = [
  "analysis_freeze.json",
  "analysis_methodology.md",
  "analysis_run_manifest.json",
  "analysis_summary.json",
  "attitude_variant_comparisons.csv",
  "calibration_metrics.csv",
  "evaluation_metrics.csv",
  "gps_yaw90_calibration_metrics.csv",
  "gps_yaw90_evaluation_metrics.csv",
  "path_selection.json",
];
console.log("STAGE 1: analysis source hashes begin");
const expectedHashes = await expectedAnalysisHashes();
const actualHashes = new Map();
for (const name of selectedInputNames) {
  const actual = await sha256(path.join(analysis, name));
  actualHashes.set(name, actual);
  assert(actual === expectedHashes.get(name), "Analysis checksum mismatch: " + name);
}
console.log("STAGE 1: verified " + actualHashes.size + " analysis files");

const freeze = JSON.parse(await fs.readFile(path.join(analysis, "analysis_freeze.json"), "utf8"));
const freezeHash = await sha256(path.join(analysis, "analysis_freeze.json"));
const analysisSummary = JSON.parse(await fs.readFile(path.join(analysis, "analysis_summary.json"), "utf8"));
const pathSelection = JSON.parse(await fs.readFile(path.join(analysis, "path_selection.json"), "utf8"));
const runStatus = JSON.parse(await fs.readFile(path.join(campaign, "run_status.json"), "utf8"));
const qcSummary = JSON.parse(await fs.readFile(path.join(campaign, "qc_summary.json"), "utf8"));
const sourceWorkbookHash = await sha256(sourceWorkbookPath);
const solverHash = await sha256(path.join(repo, "apps/Simulation/campaigns/stereo_v1_1/calibration_solver.py"));
const analyzerHash = await sha256(path.join(repo, "apps/Simulation/campaigns/stereo_v1_1/analyze_campaign.py"));

assert(analysisSummary.analysis_id === analysisId, "Analysis ID mismatch.");
assert(analysisSummary.analysis_freeze_sha256 === freezeHash, "Summary/freeze hash mismatch.");
assert(freeze.selected_path_id === "P2", "Unexpected selected path.");
assert(freeze.mc_trial_count === 100, "Unexpected frozen MC trial count.");
assert(freeze.numeric_pass_threshold === null, "Unexpected numeric pass threshold.");
assert(freeze.algorithm_sha256["calibration_solver.py"] === solverHash, "Current calibration solver differs from the frozen hash.");
assert(freeze.algorithm_sha256["analyze_campaign.py"] === analyzerHash, "Current evaluator differs from the frozen hash.");
assert(freeze.robust_loss.scope === "image residuals only", "Frozen robust-loss scope changed.");
assert(freeze.sensitivity_case_count === 50000, "Unexpected sensitivity case count.");
assert(qcSummary.planned_run_count === 39 && qcSummary.valid_run_count === 39, "Unexpected source QC counts.");
assert(runStatus.run_slots.length === 39, "Unexpected number of run slots.");

console.log("STAGE 2: source workbook import begin");
const sourceWorkbook = await SpreadsheetFile.importXlsx(await FileBlob.load(sourceWorkbookPath));
console.log("STAGE 2: source workbook import complete");
const sourceSheetNames = sourceWorkbook.worksheets.items.map((sheet) => sheet.name);
assert(JSON.stringify(sourceSheetNames) === JSON.stringify([
  "00_요약", "01_실행현황", "02_경로추종", "03_관측품질",
  "04_교정비교", "05_그래프", "98_출처", "99_정의",
]), "Source workbook sheet list/order differs from the observed baseline.");
const originalSnapshots = new Map(sourceWorkbook.worksheets.items.map((sheet) => [sheet.name, snapshotSheet(sheet)]));
const originalGraph = sourceWorkbook.worksheets.getItem("05_그래프");
const chartInspection = await sourceWorkbook.inspect({ kind: "drawing", sheetId: "05_그래프", maxChars: 100000 });
const originalChartRows = chartRecords(chartInspection).filter((record) => record.drawingType === "chart");
const originalChartTitles = originalChartRows.map((record) => sourceWorkbook.resolve(record.id).title.text);
const originalChartData = chartDataSignature(sourceWorkbook, originalChartRows);
const graphOriginalRowNumbers = {
  pairCountTitle: 18,
  v0Title: 91,
  pairedTitle: 182,
};
assert(String(originalGraph.getRange("A18").values[0][0]).includes("기본 all_data 짝 비교 사례 수"),
  "Expected paired-count title is not in 05_그래프!A18.");
assert(String(originalGraph.getRange("A91").values[0][0]).startsWith("V0 홀드아웃: 경로별 3D RMSE"),
  "Expected V0 path title is not in 05_그래프!A91.");
assert(String(originalGraph.getRange("A182").values[0][0]).startsWith("RPY_IMU 대 YAW_ONLY: 비교 가능 사례"),
  "Expected paired-share title is not in 05_그래프!A182.");
const originalPairCounts = originalGraph.getRange("A20:E23").values;
assert(originalPairCounts.length === 4, "Original paired comparison table is missing.");

const trackingRows = sourceWorkbook.worksheets.getItem("02_경로추종").getRange("A6:R44").values;
const trackingByRun = new Map();
for (let i = 0; i < trackingRows.length; i += 1) {
  const row = trackingRows[i];
  if (typeof row[0] !== "string") continue;
  trackingByRun.set(row[0], { sourceRow: i + 6, group: row[1], path: row[2], p95: row[12] });
}
assert(trackingByRun.size === 39, "Expected 39 tracking rows in 02_경로추종.");

const qcRows = [];
const invalidFrameRows = [];
const attemptIntegrityCount = { checked: 0, missing: 0, mismatch: 0 };
const selectedAttemptIds = [];
const axisContractViolations = [];
let imageInvalidTotal = 0;
let scienceInvalidTotal = 0;
let invalidFrameWithoutPairTotal = 0;
let gtWarningCount = 0;
let oldP95WarningCount = 0;
let p95AboveThresholdCount = 0;

console.log("STAGE 3: QC and frame-level GT reconciliation begin");
for (const slot of runStatus.run_slots) {
  const runId = slot.run_id;
  const attemptPathRel = slot.recording_path;
  const attemptDir = path.join(campaign, attemptPathRel);
  const attemptId = path.basename(attemptPathRel);
  const runParts = parseRun(runId);
  const qcPath = path.join(attemptDir, "qc_report.json");
  const attemptChecksums = checksumEntries(await fs.readFile(path.join(attemptDir, "checksums.sha256"), "utf8"));
  const checkedNames = [
    "qc_report.json", "gt_alignment_summary.json", "events.jsonl",
    "image_index.jsonl", "flight_reference.json",
  ];
  for (const name of checkedNames) {
    const relativeFile = name;
    const expected = attemptChecksums.get(relativeFile);
    const filePath = path.join(attemptDir, name);
    if (!expected) {
      attemptIntegrityCount.missing += 1;
      continue;
    }
    attemptIntegrityCount.checked += 1;
    if (await sha256(filePath) !== expected) attemptIntegrityCount.mismatch += 1;
  }
  const qc = JSON.parse(await fs.readFile(qcPath, "utf8"));
  const gtSummary = JSON.parse(await fs.readFile(path.join(attemptDir, "gt_alignment_summary.json"), "utf8"));
  const events = parseJsonLines(await fs.readFile(path.join(attemptDir, "events.jsonl"), "utf8"));
  const starts = events.filter((event) => event.event === "science_start");
  const ends = events.filter((event) => event.event === "science_end");
  assert(starts.length === 1 && ends.length === 1, "Expected one science start/end for " + runId);
  assert(qc.run_id === runId && qc.attempt_id === attemptId, "QC identity mismatch for " + runId);
  assert(qc.status === "VALID_WITH_WARNINGS" && qc.counted_in_campaign === true, "Selected attempt is not valid for " + runId);
  assert(slot.status === "VALID_WITH_WARNINGS", "Run status differs for " + runId);
  const slotAttempt = slot.attempts.find((attempt) => attempt.attempt_id === attemptId);
  assert(slotAttempt !== undefined, "Selected attempt absent from run_status for " + runId);
  const seedFlags = slotAttempt.seed_effective || {};
  const scienceStart = starts[0];
  const scienceEnd = ends[0];
  assert(scienceStart.simulation_epoch === scienceEnd.simulation_epoch, "Science event epoch mismatch for " + runId);
  const frameRel = "파생_재처리/" + runId + "/" + attemptId + "/gt-alignment/gt_at_images.jsonl";
  const framePath = path.join(campaign, frameRel);
  const frameHash = await sha256(framePath);
  const statusCounts = {};
  const wantedFrameIds = new Set();
  const invalidRowsForRun = [];
  let frameCount = 0;
  let invalidCount = 0;
  let scienceInvalidCount = 0;
  const frameReader = readline.createInterface({
    input: createReadStream(framePath, { encoding: "utf8" }),
    crlfDelay: Infinity,
  });
  for await (const line of frameReader) {
    if (!line) continue;
    const row = JSON.parse(line);
    frameCount += 1;
    statusCounts[row.alignment_status] = (statusCounts[row.alignment_status] || 0) + 1;
    if (row.run_id !== runId || row.attempt_id !== attemptId) {
      throw new Error("GT frame identity mismatch in " + frameRel);
    }
    if (row.gt_position_frame !== "gazebo_world_enu" || row.gt_position_axis_order !== "east,north,up") {
      axisContractViolations.push(runId + "/" + row.frame_id);
    }
    if (row.alignment_status === "EXACT" || row.alignment_status === "INTERPOLATED") continue;
    invalidCount += 1;
    wantedFrameIds.add(row.frame_id);
    const sameEpoch = row.simulation_epoch === scienceStart.simulation_epoch;
    const inScience = sameEpoch
      && row.image_timestamp_ns >= scienceStart.simulation_timestamp_ns
      && row.image_timestamp_ns <= scienceEnd.simulation_timestamp_ns;
    if (inScience) scienceInvalidCount += 1;
    invalidRowsForRun.push({
      runId,
      attemptId,
      cameraId: row.camera_id,
      frameId: row.frame_id,
      simulationEpoch: row.simulation_epoch,
      imageTimestampNs: row.image_timestamp_ns,
      scienceStartNs: scienceStart.simulation_timestamp_ns,
      scienceEndNs: scienceEnd.simulation_timestamp_ns,
      scienceEpoch: scienceStart.simulation_epoch,
      sciencePhase: !sameEpoch ? "시뮬레이션 epoch 다름"
        : inScience ? "연구 구간" : row.image_timestamp_ns < scienceStart.simulation_timestamp_ns
          ? "연구 구간 전" : "연구 구간 후",
      alignmentStatus: row.alignment_status,
      gtBeforeNs: row.gt_before_timestamp_ns,
      gtAfterNs: row.gt_after_timestamp_ns,
      gtGapNs: row.gt_bracketing_gap_ns,
      pairId: "",
      imageIndexFound: false,
      sourceFile: frameRel,
    });
  }
  const invalidRowsById = new Map(invalidRowsForRun.map((row) => [row.frameId, row]));
  const summaryCounts = gtSummary.image_count_by_alignment_status || {};
  assert(frameCount === gtSummary.image_count_total, "GT image count mismatch for " + runId);
  assert(JSON.stringify(Object.fromEntries(Object.entries(statusCounts).sort()))
    === JSON.stringify(Object.fromEntries(Object.entries(summaryCounts).sort())),
  "GT alignment status count mismatch for " + runId);
  assert(scienceInvalidCount === 0, "Invalid GT image occurs during science interval for " + runId);
  const imageIndexPath = path.join(attemptDir, "image_index.jsonl");
  const imageReader = readline.createInterface({
    input: createReadStream(imageIndexPath, { encoding: "utf8" }),
    crlfDelay: Infinity,
  });
  for await (const line of imageReader) {
    if (!line) continue;
    const row = JSON.parse(line);
    if (wantedFrameIds.has(row.frame_id)) {
      const target = invalidRowsById.get(row.frame_id);
      if (target) {
        target.imageIndexFound = true;
        target.pairId = row.pair_id || "";
      }
    }
  }
  assert(invalidRowsForRun.every((row) => row.imageIndexFound), "Invalid frame lacks an image-index row in " + runId);
  invalidFrameWithoutPairTotal += invalidRowsForRun.filter((row) => row.pairId === "").length;

  const tracking = trackingByRun.get(runId);
  assert(tracking !== undefined, "Tracking P95 missing in source workbook for " + runId);
  assert(typeof tracking.p95 === "number" && Number.isFinite(tracking.p95), "Tracking P95 is not numeric for " + runId);
  if (tracking.p95 > 1.0) p95AboveThresholdCount += 1;
  const p95Warn = qc.warnings.includes("tracking_position_p95_above_configured_limit");
  const gtWarn = qc.warnings.includes("some_images_lack_valid_gt");
  if (p95Warn) oldP95WarningCount += 1;
  if (gtWarn) gtWarningCount += 1;
  imageInvalidTotal += invalidCount;
  scienceInvalidTotal += scienceInvalidCount;
  selectedAttemptIds.push(attemptId);
  const statusText = Object.entries(summaryCounts)
    .sort((a, b) => a[0].localeCompare(b[0]))
    .map(([name, count]) => name + "=" + count).join("; ");
  const warningText = qc.warnings.join("; ");
  qcRows.push({
    runId,
    attemptId,
    rig: runParts.rig,
    path: runParts.path,
    sourceRow: tracking.sourceRow,
    p95: tracking.p95,
    qcStatus: qc.status,
    originalP95Warning: p95Warn ? "예" : "아니오",
    originalGtWarning: gtWarn ? "예" : "아니오",
    invalidFrameCount: invalidCount,
    scienceInvalidCount,
    scienceStartNs: scienceStart.simulation_timestamp_ns,
    scienceEndNs: scienceEnd.simulation_timestamp_ns,
    gtStatusCounts: statusText,
    frameFile: frameRel,
    frameHash,
    frameCount,
    sourceAllImageInvalidCount: Number(summaryCounts.EXTRAPOLATION || 0),
    px4RngVerified: seedFlags.px4_sensor_rng_verified === true ? "예" : "아니오",
    independentVerified: seedFlags.independent_repetition_verified === true ? "예" : "아니오",
    warnings: warningText,
  });
  invalidFrameRows.push(...invalidRowsForRun);
}

assert(attemptIntegrityCount.checked === 195, "Unexpected selected-attempt integrity coverage.");
assert(attemptIntegrityCount.missing === 0 && attemptIntegrityCount.mismatch === 0, "Selected attempt checksum verification failed.");
assert(qcRows.length === 39, "Expected 39 accepted attempts.");
assert(gtWarningCount === 34, "Unexpected GT warning count.");
assert(oldP95WarningCount === 31, "Unexpected old tracking P95 warning count.");
assert(p95AboveThresholdCount === 39, "Expected 39 tracking P95 values above 1 m.");
assert(invalidFrameRows.length === 9788 && imageInvalidTotal === 9788, "Unexpected invalid frame count.");
assert(scienceInvalidTotal === 0, "Unexpected science interval invalid GT count.");
assert(axisContractViolations.length === 0, "Coordinate contract violation in GT frame files.");
assert(new Set(selectedAttemptIds).size === 4, "Unexpected selected attempt identity set.");
console.log("STAGE 3: QC/GT complete, invalid frames=" + invalidFrameRows.length);

const groups = new Map();
const expectedSourceRows = new Map([
  ["evaluation_metrics.csv", 398200],
  ["gps_yaw90_evaluation_metrics.csv", 68200],
]);

async function collectEvaluationMetrics(fileName, resultFamily) {
  const filePath = path.join(analysis, fileName);
  let rowCount = 0;
  const evaluationRuns = new Set();
  for await (const row of csvRows(filePath)) {
    rowCount += 1;
    if (!/^G[01]-V0-R\d+$/.test(row.evaluation_run_id)) {
      throw new Error("Unexpected evaluation ID in " + fileName + ": " + row.evaluation_run_id);
    }
    evaluationRuns.add(row.evaluation_run_id);
    const calibration = parseRun(row.calibration_run_id);
    if (calibration.path === "V0") throw new Error("Calibration path unexpectedly V0.");
    const keyParts = [
      resultFamily, calibration.rig, calibration.path, row.calibration_run_id,
      row.evaluation_run_id, row.yaw_case, row.data_scope, row.method, row.attitude_variant,
    ];
    const key = JSON.stringify(keyParts);
    let group = groups.get(key);
    if (!group) {
      group = {
        family: resultFamily,
        rig: calibration.rig,
        path: calibration.path,
        calibrationRun: row.calibration_run_id,
        evaluationRun: row.evaluation_run_id,
        yawCase: row.yaw_case,
        scope: row.data_scope,
        method: row.method,
        attitude: row.attitude_variant,
        trials: new Set(),
        statuses: new Map(),
        rmse: [],
        p95: [],
        valid: [],
        detection: [],
      };
      groups.set(key, group);
    }
    const trial = Number(row.trial_index);
    if (!Number.isInteger(trial) || trial < 0 || trial >= 100) {
      throw new Error("Unexpected trial_index in " + fileName + ": " + row.trial_index);
    }
    if (group.trials.has(trial)) throw new Error("Duplicate MC trial in metric group " + key);
    group.trials.add(trial);
    group.statuses.set(row.evaluation_status, (group.statuses.get(row.evaluation_status) || 0) + 1);
    group.rmse.push(numberOrNull(row.position_3d_rmse_m));
    group.p95.push(numberOrNull(row.position_3d_p95_m));
    group.valid.push(numberOrNull(row.valid_measurement_rate));
    group.detection.push(numberOrNull(row.detection_rate));
  }
  assert(rowCount === expectedSourceRows.get(fileName), "Unexpected source metric row count in " + fileName);
  assert(evaluationRuns.size === 8, "Expected eight V0 evaluation runs in " + fileName);
  return rowCount;
}

console.log("STAGE 4: V0 MC CSV aggregation begin");
const existingRows = await collectEvaluationMetrics("evaluation_metrics.csv", "EXISTING_YAW branch");
const gpsRows = await collectEvaluationMetrics("gps_yaw90_evaluation_metrics.csv", "GPS_YAW90");
const expectedEvaluationRuns = [
  "G0-V0-R01", "G0-V0-R02", "G0-V0-R03", "G0-V0-R04", "G0-V0-R05",
  "G1-V0-R01", "G1-V0-R02", "G1-V0-R03",
];
for (const group of groups.values()) {
  assert(group.trials.size === 100, "MC group does not have 100 unique trials.");
  assert(group.statuses.size === 1 && group.statuses.get("EVALUATED") === 100, "Unexpected evaluation statuses.");
  assert(group.rmse.every(Number.isFinite) && group.p95.every(Number.isFinite)
    && group.valid.every(Number.isFinite) && group.detection.every(Number.isFinite),
  "A V0 MC group has missing metrics.");
}
assert(groups.size === 4664, "Unexpected V0 MC summary group count.");
assert(existingRows + gpsRows === 466400, "Unexpected total V0 metric rows.");
console.log("STAGE 4: V0 MC aggregation complete, rows=" + (existingRows + gpsRows) + ", groups=" + groups.size);

const groupRows = [...groups.values()].map((group) => {
  const rmse = stats(group.rmse);
  const p95 = stats(group.p95);
  const valid = stats(group.valid);
  const detection = stats(group.detection);
  return {
    family: group.family,
    rig: group.rig,
    path: group.path,
    calibrationRun: group.calibrationRun,
    evaluationRun: group.evaluationRun,
    yawCase: group.yawCase,
    scope: group.scope,
    method: group.method,
    attitude: group.attitude,
    mcN: group.trials.size,
    evaluatedN: group.statuses.get("EVALUATED") || 0,
    rmse,
    p95,
    valid,
    detection,
    statusText: [...group.statuses.entries()].map(([name, count]) => name + "=" + count).join("; "),
  };
});
groupRows.sort((a, b) =>
  a.family.localeCompare(b.family)
  || a.evaluationRun.localeCompare(b.evaluationRun)
  || a.calibrationRun.localeCompare(b.calibrationRun)
  || a.method.localeCompare(b.method)
  || a.attitude.localeCompare(b.attitude)
  || a.scope.localeCompare(b.scope)
  || a.yawCase.localeCompare(b.yawCase));

const pathMetricGroups = new Map();
async function collectPathComparisons() {
  const filePath = path.join(analysis, "evaluation_metrics.csv");
  for await (const row of csvRows(filePath)) {
    const cal = parseRun(row.calibration_run_id);
    const evalRun = parseRun(row.evaluation_run_id);
    if (!["A_rtk_independent", "B_rtk_joint"].includes(row.method)
      || row.attitude_variant !== "RPY_IMU"
      || !["all_data", "matched_200"].includes(row.data_scope)) continue;
    let branch;
    let acceptedYaw;
    if (cal.rig === "G0" && evalRun.rig === "G0" && ["P0", "P1", "P2", "P3", "P4"].includes(cal.path)) {
      branch = "G0 EXISTING_YAW (Y0-Y6)";
      acceptedYaw = /^Y[0-6]$/.test(row.yaw_case);
    } else if (cal.rig === "G1" && evalRun.rig === "G1" && ["P0", "P4"].includes(cal.path)) {
      branch = "G1 primary-zero";
      acceptedYaw = row.yaw_case === "G1_PRIMARY_ZERO";
    } else {
      continue;
    }
    if (!acceptedYaw) continue;
    const key = JSON.stringify([branch, cal.rig, cal.path, row.calibration_run_id, row.evaluation_run_id, row.method, row.data_scope]);
    let pathGroup = pathMetricGroups.get(key);
    if (!pathGroup) {
      pathGroup = {
        branch, rig: cal.rig, path: cal.path, calibrationRun: row.calibration_run_id,
        evaluationRun: row.evaluation_run_id, method: row.method, scope: row.data_scope,
        byTrial: new Map(), rawRmse: [], rawP95: [], rawValid: [], rawDetection: [],
        yawCases: new Map(),
      };
      pathMetricGroups.set(key, pathGroup);
    }
    const trial = Number(row.trial_index);
    let trialGroup = pathGroup.byTrial.get(trial);
    if (!trialGroup) {
      trialGroup = { rmse: [], p95: [], valid: [], detection: [], yawCases: new Set() };
      pathGroup.byTrial.set(trial, trialGroup);
    }
    if (trialGroup.yawCases.has(row.yaw_case)) throw new Error("Duplicate yaw case in path MC group.");
    trialGroup.yawCases.add(row.yaw_case);
    trialGroup.rmse.push(numberOrNull(row.position_3d_rmse_m));
    trialGroup.p95.push(numberOrNull(row.position_3d_p95_m));
    trialGroup.valid.push(numberOrNull(row.valid_measurement_rate));
    trialGroup.detection.push(numberOrNull(row.detection_rate));
    pathGroup.rawRmse.push(numberOrNull(row.position_3d_rmse_m));
    pathGroup.rawP95.push(numberOrNull(row.position_3d_p95_m));
    pathGroup.rawValid.push(numberOrNull(row.valid_measurement_rate));
    pathGroup.rawDetection.push(numberOrNull(row.detection_rate));
    pathGroup.yawCases.set(row.yaw_case, (pathGroup.yawCases.get(row.yaw_case) || 0) + 1);
  }
}
console.log("STAGE 5: V0 path comparison begin");
await collectPathComparisons();
console.log("STAGE 5: V0 path comparison complete, pairs=" + pathMetricGroups.size);

const runPathRows = [];
const pathSummaryMap = new Map();
for (const group of pathMetricGroups.values()) {
  assert(group.byTrial.size === 100, "Path rollup is missing an MC trial.");
  const meansByTrial = { rmse: [], p95: [], valid: [], detection: [] };
  for (const trialGroup of group.byTrial.values()) {
    const expectedYawCount = group.rig === "G0" ? 7 : 1;
    assert(trialGroup.yawCases.size === expectedYawCount, "Unexpected yaw coverage in path rollup.");
    for (const metric of ["rmse", "p95", "valid", "detection"]) {
      meansByTrial[metric].push(trialGroup[metric].reduce((sum, value) => sum + value, 0) / trialGroup[metric].length);
    }
  }
  const runRow = {
    ...group,
    mcTrials: group.byTrial.size,
    yawCaseCount: group.rig === "G0" ? 7 : 1,
    caseMetricCount: group.rawRmse.length,
    rmseTrialMeans: stats(meansByTrial.rmse),
    p95TrialMeans: stats(meansByTrial.p95),
    validTrialMeans: stats(meansByTrial.valid),
    detectionTrialMeans: stats(meansByTrial.detection),
    rmseCaseMean: stats(group.rawRmse).mean,
    p95CaseMean: stats(group.rawP95).mean,
    validCaseMean: stats(group.rawValid).mean,
    detectionCaseMean: stats(group.rawDetection).mean,
  };
  runPathRows.push(runRow);
  const summaryKey = JSON.stringify([group.branch, group.rig, group.path, group.method, group.scope]);
  let aggregate = pathSummaryMap.get(summaryKey);
  if (!aggregate) {
    aggregate = {
      branch: group.branch,
      rig: group.rig,
      path: group.path,
      method: group.method,
      scope: group.scope,
      calibrationRuns: new Set(),
      evaluationRuns: new Set(),
      caseMetrics: { rmse: [], p95: [], valid: [], detection: [] },
      runTrialMeans: { rmse: [], p95: [], valid: [], detection: [] },
    };
    pathSummaryMap.set(summaryKey, aggregate);
  }
  aggregate.calibrationRuns.add(group.calibrationRun);
  aggregate.evaluationRuns.add(group.evaluationRun);
  for (const metric of ["rmse", "p95", "valid", "detection"]) {
    const values = metric === "rmse" ? group.rawRmse
      : metric === "p95" ? group.rawP95
        : metric === "valid" ? group.rawValid : group.rawDetection;
    aggregate.caseMetrics[metric].push(...values);
    aggregate.runTrialMeans[metric].push(...meansByTrial[metric]);
  }
}

const pathSummaryRows = [...pathSummaryMap.values()].map((group) => ({
  ...group,
  caseMetricCount: group.caseMetrics.rmse.length,
  runTrialMeanCount: group.runTrialMeans.rmse.length,
  rmse: stats(group.caseMetrics.rmse),
  p95: stats(group.caseMetrics.p95),
  valid: stats(group.caseMetrics.valid),
  detection: stats(group.caseMetrics.detection),
  runTrialRmse: stats(group.runTrialMeans.rmse),
})).sort((a, b) =>
  a.rig.localeCompare(b.rig) || a.path.localeCompare(b.path)
  || a.method.localeCompare(b.method) || a.scope.localeCompare(b.scope));

const graphMethodRows = originalPairCounts.slice(1).map((row) => ({
  method: row[0], improved: Number(row[1]), regressed: Number(row[2]),
  unchanged: Number(row[3]), notComparable: Number(row[4]),
}));
const pairedComparisonRows = [];
for await (const row of csvRows(path.join(analysis, "attitude_variant_comparisons.csv"))) {
  if (row.family !== "primary") continue;
  if (row.condition_kind !== "yaw_case") continue;
  if (row.data_scope !== "all_data") continue;
  if (row.condition_id === "GPS_YAW90") continue;
  let aggregate = pairedComparisonRows.find((entry) => entry.method === row.method);
  if (!aggregate) {
    aggregate = { method: row.method, improved: 0, regressed: 0, unchanged: 0, notComparable: 0 };
    pairedComparisonRows.push(aggregate);
  }
  if (row.outcome === "IMPROVED") aggregate.improved += 1;
  else if (row.outcome === "REGRESSED") aggregate.regressed += 1;
  else if (row.outcome === "UNCHANGED") aggregate.unchanged += 1;
  else if (row.outcome === "NOT_COMPARABLE") aggregate.notComparable += 1;
}
const graphPairMap = new Map(graphMethodRows.map((row) => [row.method, row]));
for (const row of pairedComparisonRows) {
  const original = graphPairMap.get(row.method);
  assert(original !== undefined, "Original comparison table has an unknown method.");
  assert(original.improved === row.improved && original.regressed === row.regressed
    && original.unchanged === row.unchanged && original.notComparable === row.notComparable,
  "Existing-yaw paired outcome counts do not match the source CSV.");
}
assert(pairedComparisonRows.length === 3, "Expected three existing-yaw comparison methods.");

const gpsPairedGroups = new Map();
for await (const row of csvRows(path.join(analysis, "attitude_variant_comparisons.csv"))) {
  if (row.family !== "primary" || row.condition_kind !== "yaw_case"
    || row.condition_id !== "GPS_YAW90") continue;
  const key = JSON.stringify([row.method, row.data_scope]);
  let aggregate = gpsPairedGroups.get(key);
  if (!aggregate) {
    aggregate = {
      method: row.method, scope: row.data_scope,
      improved: 0, regressed: 0, unchanged: 0, notComparable: 0,
    };
    gpsPairedGroups.set(key, aggregate);
  }
  if (row.outcome === "IMPROVED") aggregate.improved += 1;
  else if (row.outcome === "REGRESSED") aggregate.regressed += 1;
  else if (row.outcome === "UNCHANGED") aggregate.unchanged += 1;
  else if (row.outcome === "NOT_COMPARABLE") aggregate.notComparable += 1;
  else throw new Error("Unknown paired comparison outcome: " + row.outcome);
}
const gpsPairedRows = [...gpsPairedGroups.values()].sort((a, b) =>
  a.method.localeCompare(b.method) || a.scope.localeCompare(b.scope));
assert(gpsPairedRows.length === 6, "Expected three methods × two scopes for GPS_YAW90 pairs.");
console.log("STAGE 6: paired attitude outcomes complete");

const baseProtocolCases = (25 * 7 * 2 + 6 * 1 * 2) * 100;
const gpsExtensionCases = (25 * 1 * 2 + 6 * 1 * 2) * 100;
assert(baseProtocolCases === 36200 && gpsExtensionCases === 6200
  && baseProtocolCases + gpsExtensionCases === 42400, "Primary case-count decomposition failed.");
assert(analysisSummary.calibration_batch_count === 42400, "Primary case count differs from summary.");
assert(pathSummaryRows.length === 28, "Unexpected V0 route-comparison summary coverage.");

const review = sourceWorkbook.worksheets.add("06_리뷰판정");
console.log("STAGE 7: review workbook sheets and tables begin");
const qcSheet = sourceWorkbook.worksheets.add("07_사후QC");
const mcSheet = sourceWorkbook.worksheets.add("08_V0_MC분포");
const frameSheet = sourceWorkbook.worksheets.add("09_GT제외프레임");
const formulaSheet = sourceWorkbook.worksheets.add("10_계산추적");

review.showGridLines = false;
qcSheet.showGridLines = false;
mcSheet.showGridLines = false;
frameSheet.showGridLines = false;
formulaSheet.showGridLines = false;

review.getRange("A1:M1").merge();
review.getRange("A1").values = [["v1.2 결과 사후검토 · 동결 분석 재집계"]];
review.getRange("A1:M1").format = {
  fill: navy, font: { name: font, size: 16, bold: true, color: "#FFFFFF" },
  verticalAlignment: "center",
};
review.getRange("A1:M1").format.rowHeight = 32;
review.getRange("A2:M2").merge();
review.getRange("A2").values = [[
  "분석 " + analysisId + " · 원본 workbook은 보존됨 · 이 탭은 저장된 결과를 재검산한 사후 검토입니다.",
]];
review.getRange("A2:M2").format = {
  font: { name: font, size: 10, bold: true, color: navy },
  fill: teal, wrapText: true, verticalAlignment: "center",
};
review.getRange("A2:M2").format.rowHeight = 24;
review.getRange("A3:M3").merge();
review.getRange("A3").values = [[
  "MC 백분위와 표준편차는 각 조건의 100개 trial 결과를 요약한 경험적 분포입니다. PX4 난수 독립성과 비행 반복 독립성이 검증되지 않아 물리 비행의 신뢰구간으로 해석하지 않습니다.",
]];
review.getRange("A3:M3").format = {
  font: { name: font, size: 9, color: ink },
  fill: "#F4F6F8", wrapText: true, verticalAlignment: "center",
};
review.getRange("A3:M3").format.rowHeight = 30;

review.getRange("A5:B14").values = [
  ["검토 지표", "사후 재집계"],
  ["추종 위치 P95 > 1 m", null],
  ["기존 QC의 P95 초과 경고", null],
  ["기존 QC의 일부 이미지 GT 누락 경고", null],
  ["프레임 단위 무효 GT 이미지", null],
  ["연구 구간 내부 무효 GT 이미지", null],
  ["V0 MC 조건 그룹", null],
  ["조건당 100 trial인 MC 그룹", null],
  ["V0 평가 지표 원시 행 수", existingRows + gpsRows],
  ["무효 프레임 중 pair ID 없음", null],
];
review.getRange("B6:B12").formulas = [
  ["=COUNTIF('07_사후QC'!G5:G43,\"P95 경고\")"],
  ["=COUNTIF('07_사후QC'!I5:I43,\"예\")"],
  ["=COUNTIF('07_사후QC'!J5:J43,\"예\")"],
  ["=SUM('07_사후QC'!K5:K43)"],
  ["=SUM('07_사후QC'!L5:L43)"],
  ["=COUNTA('08_V0_MC분포'!A5:A4668)"],
  ["=COUNTIF('08_V0_MC분포'!J5:J4668,100)"],
];
review.getRange("B14").formulas = [["=COUNTBLANK('09_GT제외프레임'!C5:C" + (invalidFrameRows.length + 4) + ")"]];
review.getRange("A5:B5").format = {
  fill: navy, font: { name: font, size: 10, bold: true, color: "#FFFFFF" },
  horizontalAlignment: "center", verticalAlignment: "center",
};
styleBody(review, "A6:B14", 22);
review.getRange("B6:B14").format.numberFormat = "#,##0";
review.getRange("A6:A14").format.columnWidth = 38;
review.getRange("B6:B14").format.columnWidth = 20;
review.getRange("A5:B14").format.borders = { preset: "outside", style: "thin", color: "#C8D2DC" };
review.getRange("A15:D22").values = [
  ["쟁점", "재판정", "반영 내용", "남는 한계"],
  ["V0 최종 성능과 P2 근거", "일부 맞음", "요약본에 없던 run×MC 위치 RMSE/P95·검출률·유효 측정률 분포를 새 탭에 재집계. 05 그래프의 경로 평균과 CSV를 대조.", "V0는 동결된 카메라 자세로 위치를 평가합니다. V0 카메라 자세 재추정 오차는 이 설계의 평가값이 아닙니다. 수치 허용 오차 기준도 없습니다."],
  ["추종 P95 경고", "맞음", "39/39가 1 m 초과. 기존 QC는 31개만 경고했고 V0 8개가 빠져 별도 사후 flag를 추가.", "기존 attempt QC를 고치지 않았습니다. 사전 동결 시점 판단과 사후 품질 요약을 구분합니다."],
  ["일부 프레임의 GT 누락", "맞음, 프레임 파일에서 추적 가능", "34개 실행의 9,788개 frame ID와 timestamp를 추가. pair ID가 있으면 표시하고, 없는 프레임은 무대응 영상으로 공란 유지. 전부 연구 구간 전이며 구간 내 무효 GT는 0개.", "GT 프레임 파일은 원본 checksums manifest에 없어서 새 탭에 파일별 SHA-256을 기록했습니다."],
  ["MC100의 독립 반복 해석", "맞음", "4,664 조건 그룹마다 100 trial의 평균·표준편차·중앙값·2.5/97.5 백분위를 추가.", "MC trial은 독립 비행 100회가 아닙니다. 경험적 MC 분포는 비행 간 신뢰구간이 아닙니다."],
  ["42,400과 GPS_YAW90", "범위 설명이 필요", "36,200개 기존 v1.2 yaw case와 6,200개 GPS_YAW90 확장을 분리. 민감도 50,000건은 별도.", "민감도 전 조합을 재계산하지 않았습니다. 동결 결과와 저장된 metrics CSV를 재집계했습니다."],
  ["RPY/YAW 그래프의 yaw 범위", "맞음", "기존 all_data 비교는 EXISTING_YAW primary branch만 포함합니다. 제목에 GPS 제외를 명시하고 GPS_YAW90 paired chart를 추가.", "P2 GPS_YAW90 진단 그래프도 기존에 별도로 있었으며 유지했습니다."],
  ["Huber 설명", "맞음", "설명 페이지의 인쇄용 노트를 frozen solver와 일치시켜 Huber는 영상 잔차에만 적용한다고 정정.", "동결 분석 코드/실험은 변경하지 않았습니다."],
];
review.getRange("A15:D15").format = {
  fill: navy, font: { name: font, size: 9, bold: true, color: "#FFFFFF" },
  horizontalAlignment: "center", verticalAlignment: "center", wrapText: true,
};
review.getRange("A15:D15").format.rowHeight = 28;
review.getRange("A16:D22").format = {
  font: { name: font, size: 9, color: ink },
  verticalAlignment: "top", wrapText: true,
};
review.getRange("A16:D22").format.rowHeight = 64;
setColumnWidths(review, { A: 26, B: 18, C: 54, D: 54, E: 18, F: 18, G: 18, H: 18, I: 18, J: 18, K: 18, L: 18, M: 18 });
review.getRange("A24:F24").merge();
review.getRange("A24").values = [["경로 선정 기준: 동결 전에 수행한 covariance propagation 순위입니다. 5개 G0 후보 중 P2가 예측 평균 σ가 가장 낮았으나 P4와의 차이는 0.000073 m이며, 수치 합격 한계는 정의되지 않았습니다."]];
review.getRange("A24:F24").format = { fill: amber, font: { name: font, size: 9, bold: true, color: ink }, wrapText: true, verticalAlignment: "center" };
review.getRange("A24:F24").format.rowHeight = 36;
review.getRange("A26:F31").values = [
  ["사전 예측 순위", "경로", "교정 run 수", "예측 평균 위치 σ (m)", "예측 P95 위치 σ (m)", "참조 시간 평균 (s)"],
  ...pathSelection.candidate_summary.map((candidate, index) => [
    index + 1, candidate.path_id, candidate.calibration_run_count,
    candidate.predicted_position_uncertainty_mean_sigma_m,
    candidate.predicted_position_uncertainty_p95_sigma_m,
    candidate.reference_duration_mean_s,
  ]),
];
styleHeader(review, "A26:F26");
styleBody(review, "A27:F31", 20);
review.getRange("A27:A31").format.numberFormat = "0";
review.getRange("C27:C31").format.numberFormat = "0";
review.getRange("D27:E31").format.numberFormat = "0.000000";
review.getRange("F27:F31").format.numberFormat = "0.000";
addFilterTable(review, "A26:F31", "PreV0PathRankingTable");

review.getRange("A34:M34").merge();
review.getRange("A34").values = [[
  "V0 위치 결과 · EXISTING_YAW branch / G1 primary-zero · RPY_IMU · A/B · MC case 평균 (동결 후 기술 비교; 그룹별 상세는 08_V0_MC분포)",
]];
review.getRange("A34:M34").format = {
  fill: teal, font: { name: font, size: 10, bold: true, color: navy },
  wrapText: true, verticalAlignment: "center",
};
review.getRange("A34:M34").format.rowHeight = 28;
const pathHeaders = [
  "yaw 분기", "그룹", "경로", "method", "data_scope", "V0 평가 run 수",
  "교정 run 수", "run×MC 평균 수", "MC case 지표 행 수",
  "3D RMSE 평균 (m)", "P95 평균 (m)", "유효 측정률 평균", "검출률 평균",
];
const pathRows = pathSummaryRows.map((row) => [
  row.branch, row.rig, row.path, row.method, row.scope,
  row.evaluationRuns.size, row.calibrationRuns.size, row.runTrialMeanCount,
  row.caseMetricCount, row.rmse.mean, row.p95.mean, row.valid.mean, row.detection.mean,
]);
const pathTableLast = 35 + pathRows.length;
review.getRange("A35:M" + pathTableLast).values = [pathHeaders, ...pathRows];
styleHeader(review, "A35:M35");
styleBody(review, "A36:M" + pathTableLast, 19);
review.getRange("J36:K" + pathTableLast).format.numberFormat = "0.0000";
review.getRange("L36:M" + pathTableLast).format.numberFormat = "0.0%";
review.getRange("F36:I" + pathTableLast).format.numberFormat = "#,##0";
setColumnWidths(review, { A: 27, B: 8, C: 9, D: 27, E: 16, F: 14, G: 14, H: 17, I: 18, J: 18, K: 16, L: 17, M: 14 });
addFilterTable(review, "A35:M" + pathTableLast, "V0RouteOutcomeTable");
review.getRange("A" + (pathTableLast + 2) + ":M" + (pathTableLast + 2)).merge();
review.getRange("A" + (pathTableLast + 2)).values = [[
  "G0 경로 비교는 경로당 5개 대응 교정/V0 run쌍 × 7 yaw case × 100 MC = 3,500 case 지표 행을 평균합니다. G1 P0/P4 진단은 각 3개 대응 run쌍 × primary-zero × 100 MC = 300행입니다. 집계는 case-weighted 기술 비교이며 독립 비행 표본 수 또는 유의성 검정이 아닙니다.",
]];
review.getRange("A" + (pathTableLast + 2) + ":M" + (pathTableLast + 2)).format = {
  fill: "#F4F6F8", font: { name: font, size: 9, color: ink },
  wrapText: true, verticalAlignment: "center",
};
review.getRange("A" + (pathTableLast + 2) + ":M" + (pathTableLast + 2)).format.rowHeight = 38;
review.freezePanes.freezeRows(5);

qcSheet.getRange("A1:V1").merge();
qcSheet.getRange("A1").values = [["추종 P95·GT 누락 사후 점검"]];
qcSheet.getRange("A1:V1").format = {
  fill: navy, font: { name: font, size: 15, bold: true, color: "#FFFFFF" }, verticalAlignment: "center",
};
qcSheet.getRange("A1:V1").format.rowHeight = 30;
qcSheet.getRange("A2:V2").merge();
qcSheet.getRange("A2").values = [[
  "P95(m)은 원본 workbook 02_경로추종 열 M의 해당 run 값을 직접 참조합니다. P95 경고는 > 1.0 m로 사후 계산합니다. 과거 QC JSON은 덮어쓰지 않았습니다. 과학 구간 판정은 각 run의 logged science_start/end 시뮬레이션 timestamp(ns)와 프레임 image timestamp(ns)를 비교합니다.",
]];
qcSheet.getRange("A2:V2").format = { fill: teal, font: { name: font, size: 9, color: ink }, wrapText: true };
qcSheet.getRange("A2:V2").format.rowHeight = 36;
const qcHeaders = [
  "run_id", "attempt_id", "그룹", "경로", "원본 P95 셀", "추종 P95 (m)",
  "사후 P95 판정", "원본 QC 상태", "원본 P95 경고", "원본 GT 경고",
  "무효 GT 프레임 수", "연구구간 무효 GT 수", "science_start (ns)", "science_end (ns)",
  "GT alignment 상태별 수", "프레임 상세 상대경로", "프레임 파일 SHA-256",
  "프레임 행 수", "원본 summary 무효 수", "PX4 RNG 검증", "비행 반복 독립 검증",
  "원본 QC 경고 목록",
];
const qcDataRows = qcRows.map((row) => [
  row.runId, row.attemptId, row.rig, row.path, "02_경로추종!M" + row.sourceRow, null, null,
  row.qcStatus, row.originalP95Warning, row.originalGtWarning, row.invalidFrameCount,
  row.scienceInvalidCount, row.scienceStartNs, row.scienceEndNs, row.gtStatusCounts,
  row.frameFile, row.frameHash, row.frameCount, row.sourceAllImageInvalidCount,
  row.px4RngVerified, row.independentVerified, row.warnings,
]);
qcSheet.getRange("A4:V43").values = [qcHeaders, ...qcDataRows];
qcSheet.getRange("F5:F43").formulas = qcRows.map((row) => ["='02_경로추종'!M" + row.sourceRow]);
qcSheet.getRange("G5:G43").formulas = qcRows.map((_, index) => [
  "=IF(F" + (index + 5) + ">1,\"P95 경고\",\"기준 이내\")",
]);
styleHeader(qcSheet, "A4:V4");
styleBody(qcSheet, "A5:V43", 28);
qcSheet.getRange("F5:F43").format.numberFormat = "0.000";
qcSheet.getRange("K5:L43").format.numberFormat = "#,##0";
qcSheet.getRange("M5:N43").format.numberFormat = "#,##0";
qcSheet.getRange("R5:S43").format.numberFormat = "#,##0";
qcSheet.getRange("G5:G43").conditionalFormats.add("containsText", {
  text: "P95 경고",
  format: { fill: amber, font: { color: "#7A4B00", bold: true } },
});
setColumnWidths(qcSheet, {
  A: 17, B: 10, C: 8, D: 8, E: 20, F: 14, G: 16, H: 22, I: 18, J: 20,
  K: 15, L: 17, M: 19, N: 19, O: 36, P: 78, Q: 68, R: 14, S: 16,
  T: 14, U: 18, V: 72,
});
addFilterTable(qcSheet, "A4:V43", "PostFreezeQCTable");
qcSheet.freezePanes.freezeRows(4);

mcSheet.getRange("A1:AD1").merge();
mcSheet.getRange("A1").values = [["V0 평가 · 조건별 100 MC 분포"]];
mcSheet.getRange("A1:AD1").format = {
  fill: navy, font: { name: font, size: 15, bold: true, color: "#FFFFFF" }, verticalAlignment: "center",
};
mcSheet.getRange("A1:AD1").format.rowHeight = 30;
mcSheet.getRange("A2:AD2").merge();
mcSheet.getRange("A2").values = [[
  "3D 위치오차는 estimated − truth의 world ENU [east,north,up] 축 정의, 단위 m입니다. 4,664개 조건 그룹마다 trial_index 0–99의 100개 평가 결과를 요약했습니다. q2.5/q97.5는 선형 보간 경험적 Monte Carlo 분위수(Excel PERCENTILE.INC와 같은 Type 7)이고 SD는 표본 표준편차입니다. 숫자는 코드가 계산한 고정 결과이며, raw trial 값은 해시가 확인된 analysis CSV에 남아 있습니다. 비행 신뢰구간이 아닙니다.",
]];
mcSheet.getRange("A2:AD2").format = { fill: teal, font: { name: font, size: 9, color: ink }, wrapText: true };
mcSheet.getRange("A2:AD2").format.rowHeight = 40;
const mcHeaders = [
  "result_family", "그룹", "교정 경로", "calibration_run_id", "evaluation_run_id",
  "yaw_case", "data_scope", "method", "attitude_variant", "MC trial 수", "평가 완료 수",
  "3D RMSE 평균 (m)", "3D RMSE SD (m)", "3D RMSE 중앙값 (m)", "3D RMSE q2.5 (m)", "3D RMSE q97.5 (m)",
  "P95 평균 (m)", "P95 SD (m)", "P95 중앙값 (m)", "P95 q2.5 (m)", "P95 q97.5 (m)",
  "유효 측정률 평균", "유효 측정률 SD", "유효 측정률 q2.5", "유효 측정률 q97.5",
  "검출률 평균", "검출률 SD", "검출률 q2.5", "검출률 q97.5", "evaluation_status counts",
];
const mcDataRows = groupRows.map((row) => [
  row.family, row.rig, row.path, row.calibrationRun, row.evaluationRun, row.yawCase,
  row.scope, row.method, row.attitude, row.mcN, row.evaluatedN,
  row.rmse.mean, row.rmse.sd, row.rmse.median, row.rmse.q025, row.rmse.q975,
  row.p95.mean, row.p95.sd, row.p95.median, row.p95.q025, row.p95.q975,
  row.valid.mean, row.valid.sd, row.valid.q025, row.valid.q975,
  row.detection.mean, row.detection.sd, row.detection.q025, row.detection.q975,
  row.statusText,
]);
const mcLastRow = mcDataRows.length + 4;
mcSheet.getRange("A4:AD" + mcLastRow).values = [mcHeaders, ...mcDataRows];
styleHeader(mcSheet, "A4:AD4");
styleBody(mcSheet, "A5:AD" + mcLastRow, 18);
mcSheet.getRange("J5:K" + mcLastRow).format.numberFormat = "#,##0";
mcSheet.getRange("L5:U" + mcLastRow).format.numberFormat = "0.0000";
mcSheet.getRange("V5:AC" + mcLastRow).format.numberFormat = "0.0%";
setColumnWidths(mcSheet, {
  A: 24, B: 8, C: 10, D: 18, E: 18, F: 20, G: 15, H: 27, I: 17, J: 12,
  K: 12, L: 17, M: 16, N: 17, O: 16, P: 17, Q: 15, R: 14, S: 15, T: 14,
  U: 15, V: 17, W: 15, X: 16, Y: 17, Z: 15, AA: 14, AB: 15, AC: 16, AD: 24,
});
addFilterTable(mcSheet, "A4:AD" + mcLastRow, "V0MCDistributionTable");
mcSheet.freezePanes.freezeRows(4);

frameSheet.getRange("A1:M1").merge();
frameSheet.getRange("A1").values = [["GT 정렬에서 제외된 이미지 프레임"]];
frameSheet.getRange("A1:M1").format = {
  fill: navy, font: { name: font, size: 15, bold: true, color: "#FFFFFF" }, verticalAlignment: "center",
};
frameSheet.getRange("A1:M1").format.rowHeight = 30;
frameSheet.getRange("A2:M2").merge();
frameSheet.getRange("A2").values = [[
  "GT 정렬 상태가 EXACT/INTERPOLATED가 아닌 9,788개 프레임입니다. frame_id를 보존하고 연결된 pair_id가 있으면 표시합니다. pair_id 공란은 이미지가 인덱스에 없다는 뜻이 아니라 무대응 영상이라는 뜻입니다. 각 실행의 logged science_start/end(ns)와 image timestamp(ns)로 구간을 분류했습니다. 원본 GT 좌표는 변환하지 않았습니다.",
]];
frameSheet.getRange("A2:M2").format = { fill: teal, font: { name: font, size: 9, color: ink }, wrapText: true };
frameSheet.getRange("A2:M2").format.rowHeight = 36;
const frameHeaders = [
  "run_id", "attempt_id", "pair_id", "camera_id", "frame_id", "simulation_epoch",
  "image_timestamp_ns", "science_phase", "alignment_status", "GT before ns",
  "GT after ns", "GT bracket gap ns", "source GT frame file",
];
const frameDataRows = invalidFrameRows.map((row) => [
  row.runId, row.attemptId, row.pairId, row.cameraId, row.frameId, row.simulationEpoch,
  row.imageTimestampNs, row.sciencePhase, row.alignmentStatus, row.gtBeforeNs,
  row.gtAfterNs, row.gtGapNs, row.sourceFile,
]);
const frameLastRow = frameDataRows.length + 4;
frameSheet.getRange("A4:M" + frameLastRow).values = [frameHeaders, ...frameDataRows];
styleHeader(frameSheet, "A4:M4");
styleBody(frameSheet, "A5:M" + frameLastRow, 18);
frameSheet.getRange("F5:G" + frameLastRow).format.numberFormat = "0";
frameSheet.getRange("J5:L" + frameLastRow).format.numberFormat = "#,##0";
setColumnWidths(frameSheet, {
  A: 17, B: 10, C: 18, D: 12, E: 40, F: 16, G: 22, H: 20, I: 18,
  J: 18, K: 18, L: 20, M: 78,
});
addFilterTable(frameSheet, "A4:M" + frameLastRow, "GTInvalidFrameTable");
frameSheet.freezePanes.freezeRows(4);

const graph = sourceWorkbook.worksheets.getItem("05_그래프");
const newGraphTitle = "EXISTING_YAW primary only · all_data · GPS_YAW90 excluded: RPY_IMU vs YAW_ONLY outcomes";
const newShareTitle = "EXISTING_YAW primary only · all_data · GPS_YAW90 excluded: comparable RPY_IMU vs YAW_ONLY shares";
const newV0GraphTitle = "V0 holdout 3D RMSE by calibration path (A/B RPY_IMU; existing-yaw branch; GPS_YAW90 excluded)";
const newV0TableTitle = "V0 홀드아웃: 경로별 3D RMSE (A/B RPY_IMU, existing-yaw branch; GPS_YAW90 제외; 동결 후 기술 비교)";
graph.getRange("A" + graphOriginalRowNumbers.pairCountTitle).values = [[
  "기본 all_data 짝 비교 사례 수 (EXISTING_YAW primary only; GPS_YAW90 별도)",
]];
graph.getRange("A" + graphOriginalRowNumbers.pairedTitle).values = [[
  "RPY_IMU 대 YAW_ONLY: EXISTING_YAW primary all_data 비교 가능 사례 (GPS_YAW90 별도)",
]];
graph.getRange("A" + graphOriginalRowNumbers.v0Title).values = [[newV0TableTitle]];
const pairChartOldTitle = "기본 all_data 사례별 RPY 대 YAW 결과 (건)";
const shareChartOldTitle = "RPY_IMU vs YAW_ONLY: 개선/악화 비율 (비교 가능 사례만, %)";
const v0ChartOldTitle = "V0 holdout 3D RMSE by calibration path (A/B, RPY_IMU; all_data vs matched_200)";
let pairChartChanged = 0;
let shareChartChanged = 0;
let v0ChartChanged = 0;
for (const record of originalChartRows) {
  const chart = sourceWorkbook.resolve(record.id);
  const title = chart.title.text;
  if (title === pairChartOldTitle) {
    chart.title.text = newGraphTitle;
    pairChartChanged += 1;
  } else if (title === shareChartOldTitle) {
    chart.title.text = newShareTitle;
    shareChartChanged += 1;
  } else if (title === v0ChartOldTitle) {
    chart.title.text = newV0GraphTitle;
    v0ChartChanged += 1;
  }
}
assert(pairChartChanged === 1 && shareChartChanged === 1 && v0ChartChanged === 1, "Expected graph titles were not changed exactly once.");

const gpsTitleRow = 224;
graph.getRange("A" + gpsTitleRow + ":H" + gpsTitleRow).merge();
graph.getRange("A" + gpsTitleRow).values = [[
  "GPS_YAW90 paired comparison: RPY_IMU vs YAW_ONLY (comparable-pair denominator; by method and scope)",
]];
graph.getRange("A" + gpsTitleRow + ":H" + gpsTitleRow).format = {
  font: { name: font, size: 11, bold: true, color: navy },
  verticalAlignment: "center",
};
graph.getRange("A" + gpsTitleRow + ":G" + gpsTitleRow).format.rowHeight = 24;
const gpsHeaders = [
  "method · scope", "comparable pairs", "IMPROVED", "REGRESSED", "UNCHANGED",
  "NOT_COMPARABLE", "IMPROVED share", "REGRESSED share",
];
const gpsRowsData = gpsPairedRows.map((row) => [
  row.method + " · " + row.scope,
  row.improved + row.regressed + row.unchanged,
  row.improved,
  row.regressed,
  row.unchanged,
  row.notComparable,
  null,
  null,
]);
graph.getRange("A225:H231").values = [gpsHeaders, ...gpsRowsData];
graph.getRange("G226:H231").formulas = gpsPairedRows.map((_, index) => {
  const rowNumber = index + 226;
  return [
    "=IFERROR(C" + rowNumber + "/(C" + rowNumber + "+D" + rowNumber + "+E" + rowNumber + "),\"\")",
    "=IFERROR(D" + rowNumber + "/(C" + rowNumber + "+D" + rowNumber + "+E" + rowNumber + "),\"\")",
  ];
});
styleHeader(graph, "A225:H225");
styleBody(graph, "A226:H231", 19);
graph.getRange("G226:H231").format.numberFormat = "0.0%";
graph.getRange("B226:F231").format.numberFormat = "#,##0";
graph.getRange("A225:H231").format.borders = { preset: "outside", style: "thin", color: "#C8D2DC" };
graph.getRange("A225:H225").format.rowHeight = 30;
const lastChartEndRow = Math.max(...originalChartRows.map((record) => record.anchor.to.row + 1));
const newChartTop = lastChartEndRow + 4;
const gpsChart = graph.charts.add("bar", [
  graph.getRange("A225:A231"),
  graph.getRange("G225:G231"),
  graph.getRange("H225:H231"),
]);
gpsChart.title = "GPS_YAW90 paired comparison: RPY_IMU vs YAW_ONLY, comparable pairs (%)";
gpsChart.titleTextStyle.fontSize = 11;
gpsChart.titleTextStyle.typeface = font;
gpsChart.legend = { position: "top", textStyle: { typeface: font, fontSize: 8 } };
gpsChart.xAxis = { axisType: "textAxis", textStyle: { typeface: font, fontSize: 8 } };
gpsChart.yAxis = { numberFormatCode: "0%", numberFormatSourceLinked: false, textStyle: { typeface: font, fontSize: 8 } };
gpsChart.xAxis.title.text = "method · data_scope";
gpsChart.yAxis.title.text = "share of comparable paired outcomes";
gpsChart.setPosition("H" + newChartTop, "Q" + (newChartTop + 16));

review.getRange("A" + (pathTableLast + 4) + ":M" + (pathTableLast + 4)).merge();
review.getRange("A" + (pathTableLast + 4)).values = [[
  "GPS_YAW90 paired attitude comparison was added to 05_그래프 from attitude_variant_comparisons.csv. Improved/regressed shares use only comparable pairs; non-comparable counts remain visible in the helper table.",
]];
review.getRange("A" + (pathTableLast + 4) + ":M" + (pathTableLast + 4)).format = {
  fill: "#F4F6F8", font: { name: font, size: 9, color: ink },
  wrapText: true, verticalAlignment: "center",
};
review.getRange("A" + (pathTableLast + 4) + ":M" + (pathTableLast + 4)).format.rowHeight = 30;

formulaSheet.getRange("A1:G1").merge();
formulaSheet.getRange("A1").values = [["계산·출처 추적표"]];
formulaSheet.getRange("A1:G1").format = {
  fill: navy, font: { name: font, size: 15, bold: true, color: "#FFFFFF" },
  verticalAlignment: "center",
};
formulaSheet.getRange("A1:G1").format.rowHeight = 30;
formulaSheet.getRange("A2:G2").merge();
formulaSheet.getRange("A2").values = [[
  "MC 통계는 분석 코드의 고정 결과로 기록했고, 단순 경고·비율은 Excel 수식으로 계산합니다. 모든 경로는 저장소 상대경로입니다. 원본 metrics CSV는 전체 trial 행을 보존합니다.",
]];
formulaSheet.getRange("A2:G2").format = { fill: teal, font: { name: font, size: 9, color: ink }, wrapText: true };
formulaSheet.getRange("A2:G2").format.rowHeight = 34;
const formulaRows = [
  ["결과/셀", "의미·단위", "계산/범위", "출처 경로", "원본 함수/contract", "셀 계산식/집계식", "hash·한계"],
  ["07_사후QC!F5:F43", "추종 위치 P95 · m", "run별 1개; source row M6:M44", sourceWorkbookRel, "02_경로추종 column M = recorded GT-to-reference position error P95", "'='02_경로추종'!M<원본행>", "원본 workbook SHA-256 " + sourceWorkbookHash],
  ["07_사후QC!G5:G43", "1m 초과 사후 경고", "39개 accepted run", "threshold: apps/Simulation/작업지시서/stereo_calibration_39run_package_v1_2/09_작업지시서_상세설명.html", "P95 > 1m", "'=IF(Frow>1,\"P95 경고\",\"기준 이내\")", "기존 QC JSON은 수정하지 않음"],
  ["08_V0_MC분포!L:AC", "3D RMSE/P95 m; world ENU [east,north,up] position error; valid/detection rate", "calibration_run × evaluation_run × yaw × scope × method × attitude; MC 0–99", analysisRel + "/evaluation_metrics.csv; " + analysisRel + "/gps_yaw90_evaluation_metrics.csv", "analyze_campaign.py::_evaluate_pose (1884–1951); _iter_metric_rows (2438–2470). Error = estimate − truth; frozen V0 policy holds camera poses fixed.", "mean; sample SD (n−1); median; type-7 q2.5/q97.5", "100 trial rows per condition; hashes " + actualHashes.get("evaluation_metrics.csv") + " / " + actualHashes.get("gps_yaw90_evaluation_metrics.csv") + "; analyzer hash " + analyzerHash],
  ["06_리뷰판정!J: M", "G0/G1 V0 path outcome means", "G0 P0–P4: 5 run pairs × 7 yaw ×100; G1 P0/P4: 3 ×1 ×100", analysisRel + "/evaluation_metrics.csv", "Selected G0 path comparison is post-freeze descriptive; G1 primary-zero rows are diagnostic", "case-weighted means by path/method/scope", "Not flight-level CI; not selection rule"],
  ["05_그래프!A226:H231", "GPS_YAW90 RPY/YAW paired outcomes", "primary family; condition_id GPS_YAW90; method × data_scope", analysisRel + "/attitude_variant_comparisons.csv", "outcome = same-case RPY_IMU minus YAW_ONLY metric comparison", "share = improved or regressed / (improved + regressed + unchanged comparable outcomes)", "NOT_COMPARABLE count shown separately; CSV SHA-256 " + actualHashes.get("attitude_variant_comparisons.csv")],
  ["09_GT제외프레임!A5:M9792", "GT 미정렬 frame IDs; pair ID when paired", "status not EXACT/INTERPOLATED; science phase by event timestamps", "파생_재처리/<run>/<attempt>/gt-alignment/gt_at_images.jsonl + image_index.jsonl", "linear ENU position/time alignment summary and recorded science_start/end events", "phase = image timestamp compared with event ns", "Blank pair ID means the indexed image had no stereo pair; no coordinate transform; per-file SHA-256 is in 07_사후QC!Q"],
  ["06_리뷰판정!A27:F31", "P0–P4 pre-V0 covariance ranking", "5 candidate paths", analysisRel + "/path_selection.json", "Local covariance propagation ranking; not a proof of global optimality", "Source values; sorted order retained", "Freeze threshold is null; P2 chosen pre-V0"],
  ["Scope count", "case IDs", "36,200 base + 6,200 GPS extension = 42,400; sensitivity = 50,000 separately", analysisRel + "/analysis_freeze.json and analysis_summary.json", "MC100 v1.2 matrix contract", "(25×7×2 + 6×1×2)×100; GPS (25×1×2 + 6×1×2)×100", "Full 2,928,820-line checksum manifest not rehashed"],
  ["Frozen algorithm", "Huber scope", "f_scale=1.345", analysisRel + "/analysis_freeze.json", "Huber applies to image residuals only; RTK/IMU Gaussian quadratic", "No analysis rerun", "calibration_solver.py SHA-256 " + solverHash],
];
const formulaLastRow = 3 + formulaRows.length;
formulaSheet.getRange("A4:G" + formulaLastRow).values = formulaRows;
styleHeader(formulaSheet, "A4:G4");
styleBody(formulaSheet, "A5:G" + formulaLastRow, 54);
formulaSheet.getRange("A5:G" + formulaLastRow).format.verticalAlignment = "top";
formulaSheet.getRange("A5:G" + formulaLastRow).format.wrapText = true;
setColumnWidths(formulaSheet, { A: 28, B: 28, C: 42, D: 78, E: 58, F: 58, G: 68 });
formulaSheet.freezePanes.freezeRows(4);

console.log("STAGE 8: workbook recalculate begin");
sourceWorkbook.recalculate();
console.log("STAGE 8: workbook recalculate complete");

const changedGraphCells = new Set([
  "A" + graphOriginalRowNumbers.pairCountTitle,
  "A" + graphOriginalRowNumbers.pairedTitle,
  "A" + graphOriginalRowNumbers.v0Title,
]);
for (const name of sourceSheetNames) {
  const original = originalSnapshots.get(name);
  const updatedSheet = sourceWorkbook.worksheets.getItem(name);
  const updatedFull = snapshotSheet(updatedSheet);
  const rangeStart = rangeStartCell(original.address);
  if (name !== "05_그래프") {
    assert(original.address === updatedFull.address, "Original sheet dimensions changed: " + name);
  }
  const updatedRange = updatedSheet.getRange(original.address);
  const originalValues = original.values;
  const updatedValues = updatedRange.values;
  const originalFormulas = original.formulas;
  const updatedFormulas = updatedRange.formulas;
  assert(JSON.stringify(originalFormulas) === JSON.stringify(updatedFormulas), "Original formulas changed: " + name);
  for (let rowIndex = 0; rowIndex < originalValues.length; rowIndex += 1) {
    for (let colIndex = 0; colIndex < originalValues[rowIndex].length; colIndex += 1) {
      const cellName = excelColumnName(rangeStart.column + colIndex) + (rangeStart.row + rowIndex);
      const isIntentionalTitleEdit = name === "05_그래프" && changedGraphCells.has(cellName);
      if (isIntentionalTitleEdit) continue;
      if (!sameCellValue(originalValues[rowIndex][colIndex], updatedValues[rowIndex][colIndex])) {
        throw new Error("Unexpected source sheet value change: " + name + "!" + cellName
          + " from " + JSON.stringify(originalValues[rowIndex][colIndex])
          + " to " + JSON.stringify(updatedValues[rowIndex][colIndex]));
      }
    }
  }
}

const sourceTitleTable = graph.getRange("A92:E99").values;
for (const pathRow of pathSummaryRows) {
  const sourceRow = sourceTitleTable.find((row) => row[0] === pathRow.rig + "-" + pathRow.path);
  if (!sourceRow) continue;
  const aColumn = pathRow.method === "A_rtk_independent"
    ? (pathRow.scope === "all_data" ? 1 : 3)
    : (pathRow.scope === "all_data" ? 2 : 4);
  assert(closeEnough(pathRow.rmse.mean, sourceRow[aColumn]), "Path V0 mean does not match existing graph for "
    + pathRow.rig + "-" + pathRow.path + " " + pathRow.method + " " + pathRow.scope);
}

const expectedNewSheetNames = ["06_리뷰판정", "07_사후QC", "08_V0_MC분포", "09_GT제외프레임", "10_계산추적"];
assert(expectedNewSheetNames.every((name) => sourceWorkbook.worksheets.items.some((sheet) => sheet.name === name)),
  "A required review sheet is missing.");
await fs.mkdir(outputDir, { recursive: true });
console.log("STAGE 9: XLSX export begin");
const xlsx = await SpreadsheetFile.exportXlsx(sourceWorkbook);
await xlsx.save(outputPath);
console.log("STAGE 9: XLSX export complete");

console.log("STAGE 10: XLSX re-import verification begin");
const roundTrip = await SpreadsheetFile.importXlsx(await FileBlob.load(outputPath));
console.log("STAGE 10: XLSX re-import complete");
const expectedAllSheetNames = [...sourceSheetNames, ...expectedNewSheetNames];
assert(JSON.stringify(roundTrip.worksheets.items.map((sheet) => sheet.name)) === JSON.stringify(expectedAllSheetNames),
  "Workbook worksheet names/order changed during export.");
for (const name of sourceSheetNames) {
  const original = originalSnapshots.get(name);
  const sheet = roundTrip.worksheets.getItem(name);
  const rangeStart = rangeStartCell(original.address);
  if (name !== "05_그래프") {
    assert(sheet.getUsedRange().address === original.address, "Original sheet dimensions changed after XLSX round-trip: " + name);
  }
  const range = sheet.getRange(original.address);
  assert(JSON.stringify(range.formulas) === JSON.stringify(original.formulas),
    "Original formulas changed after XLSX round-trip: " + name);
  const values = range.values;
  for (let rowIndex = 0; rowIndex < original.values.length; rowIndex += 1) {
    for (let colIndex = 0; colIndex < original.values[rowIndex].length; colIndex += 1) {
      const cellName = excelColumnName(rangeStart.column + colIndex) + (rangeStart.row + rowIndex);
      const isIntentionalTitleEdit = name === "05_그래프" && changedGraphCells.has(cellName);
      if (isIntentionalTitleEdit) continue;
      if (!sameCellValue(values[rowIndex][colIndex], original.values[rowIndex][colIndex])) {
        throw new Error("Original value changed after XLSX round-trip: " + name + "!" + cellName
          + " from " + JSON.stringify(original.values[rowIndex][colIndex])
          + " to " + JSON.stringify(values[rowIndex][colIndex]));
      }
    }
  }
}
const roundTripGraph = roundTrip.worksheets.getItem("05_그래프");
const roundTripDrawingRecords = chartRecords(await roundTrip.inspect({
  kind: "drawing", sheetId: "05_그래프", maxChars: 120000,
})).filter((record) => record.drawingType === "chart");
assert(roundTripDrawingRecords.length === originalChartRows.length + 1,
  "Existing chart count was not preserved plus the GPS paired chart.");
const roundTripChartTitles = roundTripDrawingRecords.map((record) => roundTrip.resolve(record.id).title.text);
const expectedChartTitles = originalChartTitles.map((title) => {
  if (title === pairChartOldTitle) return newGraphTitle;
  if (title === shareChartOldTitle) return newShareTitle;
  if (title === v0ChartOldTitle) return newV0GraphTitle;
  return title;
}).concat(["GPS_YAW90 paired comparison: RPY_IMU vs YAW_ONLY, comparable pairs (%)"]);
assert(JSON.stringify([...roundTripChartTitles].sort()) === JSON.stringify([...expectedChartTitles].sort()),
  "Chart titles changed beyond the intended scope labels and added GPS comparison.");
const roundTripChartData = chartDataSignature(roundTrip, roundTripDrawingRecords);
assert(JSON.stringify(roundTripChartData.slice(0, originalChartData.length)) === JSON.stringify(originalChartData),
  "An original chart source range changed during XLSX round-trip.");
assert(roundTripChartTitles.includes(newGraphTitle)
  && roundTripChartTitles.includes(newShareTitle)
  && roundTripChartTitles.includes(newV0GraphTitle)
  && roundTripChartTitles.includes("GPS_YAW90 paired comparison: RPY_IMU vs YAW_ONLY, comparable pairs (%)"),
"A clarified/new graph title did not survive XLSX round-trip.");
assert(JSON.stringify(roundTripGraph.getRange("A20:E23").values) === JSON.stringify(originalPairCounts),
  "Original paired outcome counts changed.");
assert(roundTripGraph.getRange("A226:H231").values.length === 6, "GPS paired graph helper rows missing.");
assert(roundTrip.worksheets.getItem("07_사후QC").getRange("A4:V43").values.length === 40, "Post-freeze QC rows missing.");
assert(roundTrip.worksheets.getItem("08_V0_MC분포").getRange("A4:AD" + mcLastRow).values.length === groupRows.length + 1,
  "MC summary rows missing after export.");
assert(roundTrip.worksheets.getItem("09_GT제외프레임").getRange("A4:M" + frameLastRow).values.length === invalidFrameRows.length + 1,
  "GT invalid frame rows missing after export.");
const summaryValues = roundTrip.worksheets.getItem("06_리뷰판정").getRange("A5:B14").values;
const summaryByLabel = new Map(summaryValues.slice(1).map((row) => [row[0], row[1]]));
assert(summaryByLabel.get("추종 위치 P95 > 1 m") === 39, "P95 summary formula did not recalculate to 39.");
assert(summaryByLabel.get("기존 QC의 P95 초과 경고") === 31, "Original P95 warning count mismatch.");
assert(summaryByLabel.get("기존 QC의 일부 이미지 GT 누락 경고") === 34, "Original GT warning count mismatch.");
assert(summaryByLabel.get("프레임 단위 무효 GT 이미지") === 9788, "Invalid GT frame summary mismatch.");
assert(summaryByLabel.get("연구 구간 내부 무효 GT 이미지") === 0, "Science-interval invalid GT summary mismatch.");
assert(summaryByLabel.get("V0 MC 조건 그룹") === 4664, "MC group summary formula mismatch.");
assert(summaryByLabel.get("조건당 100 trial인 MC 그룹") === 4664, "MC group size formula mismatch.");
assert(summaryByLabel.get("무효 프레임 중 pair ID 없음") === invalidFrameWithoutPairTotal, "Unpaired invalid frame count mismatch.");
const summaryCheck = await roundTrip.inspect({
  kind: "table", range: "06_리뷰판정!A5:B14", include: "values,formulas",
  tableMaxRows: 12, tableMaxCols: 4, maxChars: 4000,
});
await fs.mkdir(previewDir, { recursive: true });
const previewRanges = [
  ["06_리뷰판정", "A1:M" + (pathTableLast + 4), "review_summary.png"],
  ["07_사후QC", "A1:V12", "postfreeze_qc.png"],
  ["08_V0_MC분포", "A1:AD10", "v0_mc_distribution.png"],
  ["09_GT제외프레임", "A1:M10", "gt_invalid_frames.png"],
  ["10_계산추적", "A1:G13", "formula_trace.png"],
  ["05_그래프", "A178:Q204", "existing_v0_graphs.png"],
  ["05_그래프", "A304:Q331", "existing_yaw_paired_graph.png"],
  ["05_그래프", "H" + newChartTop + ":Q" + (newChartTop + 16), "gps_yaw90_paired_chart.png"],
];
for (const [sheetName, range, fileName] of previewRanges) {
  const preview = await roundTrip.render({ sheetName, range, scale: 1, format: "png" });
  await fs.writeFile(path.join(previewDir, fileName), new Uint8Array(await preview.arrayBuffer()));
}
console.log("OUTPUT=" + path.relative(repo, outputPath));
console.log("OUTPUT_SHA256=" + await sha256(outputPath));
console.log("SOURCE_WORKBOOK_SHA256=" + sourceWorkbookHash);
console.log("SOURCE_ANALYSIS_FREEZE_SHA256=" + freezeHash);
console.log("SOURCE_CODE_HASH_MATCH=" + solverHash);
console.log("SOURCE_ANALYZER_HASH_MATCH=" + analyzerHash);
console.log("SOURCE_CHECKSUMS_MATCH=" + Object.fromEntries(actualHashes));
console.log("ATTEMPT_FILE_HASHES=" + JSON.stringify(attemptIntegrityCount));
console.log("QC=" + JSON.stringify({
  runs: qcRows.length, p95AboveThreshold: p95AboveThresholdCount,
  oldP95Warnings: oldP95WarningCount, gtWarningRuns: gtWarningCount,
  invalidFrames: invalidFrameRows.length, unpairedInvalidFrames: invalidFrameWithoutPairTotal,
  scienceInvalidFrames: scienceInvalidTotal,
}));
console.log("MC=" + JSON.stringify({
  existingRows, gpsRows, totalRows: existingRows + gpsRows,
  groups: groupRows.length, trialsPerGroup: 100, evalRunIds: expectedEvaluationRuns,
}));
console.log("CASES=" + JSON.stringify({
  baseProtocolCases, gpsExtensionCases, primaryTotal: baseProtocolCases + gpsExtensionCases,
  separateSensitivityCases: freeze.sensitivity_case_count,
}));
console.log("PATH_OUTCOMES=" + JSON.stringify(pathSummaryRows.map((row) => ({
  rig: row.rig, path: row.path, method: row.method, scope: row.scope,
  caseMetrics: row.caseMetricCount, rmseMean: row.rmse.mean, p95Mean: row.p95.mean,
}))));
console.log("GPS_PAIRED=" + JSON.stringify(gpsPairedRows));
console.log("CHARTS=" + JSON.stringify({
  before: originalChartRows.length, after: roundTripDrawingRecords.length,
  original: originalChartTitles, addedAtExcelRow: newChartTop,
}));
console.log("SUMMARY_FORMULA_INSPECT=" + summaryCheck.ndjson);
console.log("OUTPUT_PRESENT=" + String((await fs.stat(outputPath)).size));
console.log("PREVIEWS=" + previewDir);

