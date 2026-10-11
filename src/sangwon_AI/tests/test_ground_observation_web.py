"""Keep stale/stopped ground evidence distinct from a flight-ready controller."""
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location(
    'native_hover_web', Path(__file__).resolve().parents[1] / 'ops/native_hover_web.py')
web = importlib.util.module_from_spec(spec)
spec.loader.exec_module(web)


class GroundObservationWebTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / 'summary.json'

    def report(self, age=0, **changes):
        row = dict(scope='continuous_disarmed_static_ev', flight_authorized=False,
                   result='streaming_ground_only', gate='ready')
        row.update(changes)
        self.path.write_text(json.dumps(row), encoding='utf-8')
        os.utime(self.path, (1000, 1000))
        return web.ground_observation_status(self.path, now=1000 + age)

    def test_does_not_invent_an_unconfigured_observer(self):
        self.assertIsNone(web.ground_observation_status(None))

    def test_missing_report_is_unavailable(self):
        self.assertEqual(web.ground_observation_status(self.path)['state'], 'unavailable')

    def test_fresh_report_is_never_flight_authorization(self):
        result = self.report(candidate={'private': 'omit'}, source_samples=[{'x': 2}])
        self.assertEqual(result['state'], 'reporting')
        self.assertFalse(result['flight_authorized'])
        self.assertNotIn('candidate', result)
        self.assertNotIn('source_samples', result)

    def test_gap_and_clock_jump_cannot_look_live(self):
        for age in (6, -1):
            self.assertEqual(self.report(age=age)['state'], 'stale')

    def test_revocation_is_terminal_even_if_report_is_recent_or_old(self):
        for age in (0, 600):
            result = self.report(age=age, result='ground_authority_revoked',
                                 gate='ground_authority_revoked')
            self.assertEqual(result['state'], 'stopped')

    def test_paused_report_does_not_claim_publication(self):
        self.assertEqual(self.report(gate='stable_timesync_required')['state'], 'paused')

    def test_wrong_scope_and_flight_claim_fail_closed(self):
        for changes in ({'scope': 'flight'}, {'flight_authorized': True}, {'gate': None}):
            self.assertEqual(self.report(**changes)['state'], 'unavailable')

    def test_corrupt_and_oversized_reports_fail_closed(self):
        for body in ('{', 'null', '{"x": NaN}', 'x' * 1_000_001):
            self.path.write_text(body, encoding='utf-8')
            self.assertEqual(web.ground_observation_status(self.path)['state'], 'unavailable')


if __name__ == '__main__':
    unittest.main()
