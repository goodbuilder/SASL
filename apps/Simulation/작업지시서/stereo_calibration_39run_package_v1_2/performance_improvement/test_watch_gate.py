"""Completion gates exercised against independently constructed receipts."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("watcher", Path(__file__).with_name("watch_and_measure.py"))
watcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(watcher)


class CompletionGateTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        watcher.HERE = root / "measurement"
        watcher.HERE.mkdir()
        for name, subdir in (("EXECUTION", "execution"), ("CAMPAIGN", "campaign"),
                             ("ANALYSIS", "analysis_test"), ("VERIFY", "verification")):
            path = root / subdir
            path.mkdir()
            setattr(watcher, name, path)

    def write(self, root, name, data):
        (root / name).write_text(json.dumps(data))

    def successful_analysis(self):
        self.write(watcher.EXECUTION, "result.json", {"exit_code": 0})
        (watcher.EXECUTION / "analysis.log").write_text('{"status":"ANALYSIS_COMPLETE"}\n')
        self.write(watcher.CAMPAIGN, "analysis_latest.json", {"analysis_id": "analysis_test"})
        for name in ("analysis_summary.json", "analysis_freeze.json", "analysis_checksums.sha256"):
            (watcher.ANALYSIS / name).write_text("fixture")

    def successful_audit(self):
        self.write(watcher.VERIFY, "result.json", {
            "status": "AUDIT_HEADING_SUMMARY_STAGES_COMPLETE", "analysis_id": "analysis_test"})
        self.write(watcher.VERIFY, "analysis_results_audit.json", {"status": "PASS"})

    def test_waits_without_receipt(self):
        self.assertFalse(watcher.completion_ready())

    def test_failed_analysis_cannot_admit_benchmark(self):
        self.write(watcher.EXECUTION, "result.json", {"exit_code": 2})
        with self.assertRaisesRegex(RuntimeError, "analysis failed"):
            watcher.completion_ready()

    def test_summary_does_not_replace_final_receipt(self):
        (watcher.ANALYSIS / "analysis_summary.json").write_text("fixture")
        self.assertFalse(watcher.completion_ready())

    def test_waits_for_audit_after_analysis(self):
        self.successful_analysis()
        self.assertFalse(watcher.completion_ready())

    def test_missing_final_checksum_manifest_rejected(self):
        self.successful_analysis()
        (watcher.ANALYSIS / "analysis_checksums.sha256").unlink()
        with self.assertRaisesRegex(RuntimeError, "missing completion artifact"):
            watcher.completion_ready()

    def test_stale_latest_pointer_rejected(self):
        self.successful_analysis()
        self.write(watcher.CAMPAIGN, "analysis_latest.json", {"analysis_id": "old"})
        with self.assertRaisesRegex(RuntimeError, "pointer mismatch"):
            watcher.completion_ready()

    def test_failed_audit_rejected(self):
        self.successful_analysis()
        self.successful_audit()
        self.write(watcher.VERIFY, "analysis_results_audit.json", {"status": "FAIL"})
        with self.assertRaisesRegex(RuntimeError, "auditor did not PASS"):
            watcher.completion_ready()

    def test_stale_success_receipt_rejected(self):
        self.successful_analysis()
        self.successful_audit()
        self.write(watcher.VERIFY, "result.json", {
            "status": "AUDIT_HEADING_SUMMARY_STAGES_COMPLETE", "analysis_id": "old"})
        with self.assertRaisesRegex(RuntimeError, "receipt version mismatch"):
            watcher.completion_ready()

    def test_both_successes_admit_measurement(self):
        self.successful_analysis()
        self.successful_audit()
        self.assertTrue(watcher.completion_ready())

    def test_partial_monitor_tail_is_not_a_completed_sample(self):
        row = {"observed_ns": 1000000000, "family_cpu_seconds": 2,
               "family_rss_bytes": 1000}
        (watcher.EXECUTION / "resource_monitor.jsonl").write_text(
            json.dumps(row) + '\n{"observed_ns":')
        self.assertEqual(watcher.baseline(), 1000000000)
        report = json.loads((watcher.HERE / "baseline.json").read_text())
        self.assertEqual(report["sample_count"], 1)


if __name__ == "__main__":
    unittest.main()
