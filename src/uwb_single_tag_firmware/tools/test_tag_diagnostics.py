"""Synthetic record tests; these do not establish RF or device performance."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import summarize_tag_diagnostics as diagnostic


def raw(**changes):
    row = dict(type='uwb_raw_cycle', schema=2, tag_id='6', diag_version=1,
               boot_id=1, session=2, seq=3, valid_mask=15, failure=[0]*4,
               first_failure=[0]*4, attempts=[1]*4, cause=[14]*4, first_cause=[14]*4,
               transaction=[1, 2, 3, 4], first_transaction=[1, 2, 3, 4])
    row.update(changes)
    return row


def attempt(**changes):
    row = dict(type='uwb_rf_attempt', schema=2, tag_id='6', diag_version=1,
               boot_id=1, session=2, seq=3, anchor=1, attempt=1, transaction=1,
               failure=4, cause=3, stage=2, start_us=100, end_us=1500,
               radio_start=0xfffffff0, radio_end=0x100, status=4,
               wait_stage=2, wait_start_us=200, wait_end_us=1500, wait_radio_start=10,
               wait_radio_end=100, deadline_radio=120, rf_progress=True, poll_tx_confirmed=True)
    row.update(changes)
    return row


def status(**changes):
    row = dict(type='uwb_raw_status', schema=2, tag_id='6', boot_id=1, session=2,
               diag_attempts=100, diag_failures=[0]*15, diag_dropped=0,
               diag_stale=0, diag_high_water=1)
    row.update(changes)
    return row


class DiagnosticSummaryTests(unittest.TestCase):
    def summarize_rows(self, *rows):
        summary = diagnostic.Summary()
        for row in rows:
            summary.ingest(row)
        return summary.result()

    def test_retry_preserves_first_failure_and_denominators(self):
        row = raw(attempts=[2, 1, 1, 1], first_failure=[4, 0, 0, 0],
                  first_cause=[3, 14, 14, 14], transaction=[5, 2, 3, 4])
        result = self.summarize_rows(row)
        self.assertEqual(result['raw']['first_success_per_opportunity'], diagnostic.ratio(3, 4))
        self.assertEqual(result['raw']['final_success_per_opportunity'], diagnostic.ratio(4, 4))
        self.assertEqual(result['raw']['ranging_attempts'], 5)
        self.assertEqual(result['raw']['retry_recovery'], diagnostic.ratio(1, 1))
        self.assertEqual(result['first_causes']['RxTimeout'], 1)
        self.assertEqual(result['final_causes']['RxTimeout'], 0)

    def test_unattempted_is_an_opportunity_not_a_wire_attempt(self):
        row = raw(valid_mask=3, attempts=[1, 1, 0, 0], failure=[0, 0, 1, 12],
                  first_failure=[0, 0, 1, 12], cause=[14, 14, 1, 13], first_cause=[14, 14, 1, 13],
                  transaction=[1, 2, 0, 0], first_transaction=[1, 2, 0, 0])
        result = self.summarize_rows(row)['raw']
        self.assertEqual(result['first_success_per_opportunity'], diagnostic.ratio(2, 4))
        self.assertEqual(result['first_success_per_attempted_anchor'], diagnostic.ratio(2, 2))
        self.assertEqual(result['unattempted_opportunities'], 2)

    def test_raw_dedup_uses_boot_session_sequence_and_reports_conflicts(self):
        row = raw()
        conflicting = raw(note='different')
        result = self.summarize_rows(row, copy.deepcopy(row), conflicting,
                                     raw(boot_id=2), raw(session=3))
        self.assertEqual(result['raw']['cycles'], 3)
        self.assertEqual(result['records']['raw_duplicates'], 2)
        self.assertEqual(result['records']['raw_conflicting_duplicates'], 1)

    def test_events_are_sparse_and_old_session_events_are_retained(self):
        result = self.summarize_rows(attempt(), attempt(),
                                     attempt(session=1, cause=6, rf_progress=False),
                                     attempt(attempt=2, transaction=5, failure=0, cause=14, stage=7))
        self.assertEqual(result['detailed_events']['received_unique'], 3)
        self.assertEqual(result['detailed_events']['successful_retries'], 1)
        self.assertIsNone(result['raw']['first_success_per_opportunity']['fraction'])
        self.assertFalse(result['physical_rf_cause_confirmed'])
        self.assertEqual(result['detailed_events']['mcu_guard_without_observed_rf_progress'], 1)

    def test_status_counters_span_sessions_but_not_boots(self):
        failures = [0]*15
        failures[3] = 4
        result = self.summarize_rows(status(), status(session=3, diag_attempts=120, diag_dropped=2,
                                                    diag_stale=1, diag_failures=failures),
                                     status(boot_id=2, diag_attempts=10))
        counters = result['heartbeat_counters']
        self.assertEqual(counters[0]['observed_nonregressing_deltas']['diag_attempts'], 20)
        self.assertEqual(counters[0]['observed_failure_deltas']['RxTimeout'], 4)
        self.assertEqual(counters[1]['observed_nonregressing_deltas'], {})
        self.assertTrue(result['diagnostic_loss_reported_cumulative'])

    def test_counter_regression_is_not_wrapped_into_a_huge_delta(self):
        result = self.summarize_rows(status(diag_attempts=0xfffffffe), status(diag_attempts=4))
        counters = result['heartbeat_counters'][0]
        self.assertEqual(counters['counter_regressions'], 1)
        self.assertEqual(counters['observed_nonregressing_deltas'], {})

    def test_envelopes_and_legacy_records_have_explicit_scope(self):
        legacy = raw(seq=4)
        del legacy['diag_version']
        result = self.summarize_rows({'value': {'message': raw()}},
                                     {'direction': 'host_to_tag', 'message': raw(seq=5)}, legacy)
        self.assertEqual(result['raw']['cycles'], 1)
        self.assertEqual(result['records']['uwb_raw_cycle_without_diag_v1'], 1)

    def test_invalid_diagnostics_do_not_partially_change_denominators(self):
        result = self.summarize_rows(raw(attempts=[2, 1, 1, 1]),
                                     raw(cause=[14, 14, 14, 3]),
                                     attempt(failure=0, cause=14),
                                     status(diag_failures=[0]*14+[1]))
        self.assertEqual(sum(result['invalid_records'].values()), 4)
        self.assertEqual(result['raw']['first_success_per_opportunity']['denominator'], 0)

    def test_recovery_timings_keep_natural_and_bench_separate(self):
        def recovery(event, at, session=2, **changes):
            row = dict(type='uwb_rf_recovery', schema=2, tag_id='6', diag_version=1,
                       boot_id=1, session=session, seq=3, event=event, reason='tx_fault_streak',
                       at_us=at, detected_us=100, next_cycle_us=0, recoveries=1)
            row.update(changes)
            return row
        result = self.summarize_rows(recovery('scheduled', 100),
                                     recovery('init_start', 1500100),
                                     recovery('init_success', 1520100, session=3),
                                     recovery('init_start', 2000000, reason='bench_request',
                                              recoveries=2, detected_us=2000000))
        self.assertEqual([r['value'] for r in result['recovery_timings']], [1500000, 20000, 0])
        self.assertEqual(result['recovery_timings'][-1]['reason'], 'bench_request')

    def test_utf8_source_hash_malformed_line_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / '원본.jsonl'
            original = (json.dumps(raw(), ensure_ascii=False)+'\n{bad\n').encode('utf-8')
            path.write_bytes(original)
            result = diagnostic.summarize([path])
            self.assertEqual(result['sources'][0]['sha256'], hashlib.sha256(original).hexdigest())
            self.assertEqual(result['records']['malformed_json_or_utf8'], 1)
            with self.assertRaises(SystemExit):
                diagnostic.main([str(path), '--output', str(path)])
            self.assertEqual(path.read_bytes(), original)
            output = Path(temp) / '요약.json'
            self.assertEqual(diagnostic.main([str(path), '--output', str(output)]), 0)
            self.assertEqual(json.loads(output.read_text(encoding='utf-8'))['raw']['cycles'], 1)
            self.assertEqual(path.read_bytes(), original)

    def test_startup_can_precede_first_radio_session(self):
        row = dict(type='uwb_rf_recovery', schema=2, tag_id='6', diag_version=1,
                   boot_id=1, session=0, seq=0, event='init_start', reason='startup',
                   at_us=1000, detected_us=1000, next_cycle_us=0, recoveries=0)
        result = self.summarize_rows(row, dict(row, event='init_success', session=2, at_us=6000))
        self.assertEqual(result['recovery_timings'][-1]['value'], 5000)
        self.assertEqual(result['invalid_records'], {})
        invalid = self.summarize_rows(dict(row, reason='mcu_guard'))
        self.assertEqual(sum(invalid['invalid_records'].values()), 1)

    def test_retry_requires_a_distinct_transaction_and_integer_version(self):
        result = self.summarize_rows(raw(attempts=[2,1,1,1], first_failure=[4,0,0,0],
                                         first_cause=[3,14,14,14]), raw(diag_version=True))
        self.assertEqual(sum(result['invalid_records'].values()), 1)
        self.assertEqual(result['records']['uwb_raw_cycle_without_diag_v1'], 1)
        self.assertEqual(result['raw']['first_success_per_opportunity']['denominator'], 0)

    def test_duplicate_keys_and_nonfinite_json_are_rejected(self):
        for line in ('{"type":"uwb_raw_cycle","type":"uwb_rf_attempt"}', '{"x":NaN}', '{"x":Infinity}'):
            with self.assertRaises(ValueError):
                diagnostic.decode_json(line)


if __name__ == '__main__':
    unittest.main()
