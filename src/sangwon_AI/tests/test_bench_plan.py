"""Bench route and preview authority boundaries."""
from pathlib import Path
import math
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
from sangwon_web.bench_plan import ROUTES, audit_view, make_plan


class BenchPlanTest(unittest.TestCase):
    def test_all_routes_hold_z_return_and_remain_synthetic(self):
        for case in ROUTES:
            plan = make_plan(case, 1.2)
            points = [t['position_m'] for t in plan['route_tasks']]
            self.assertEqual(points[-1], dict(x=0, y=0, z=1.2))
            self.assertEqual({p['z'] for p in points}, {1.2})
            self.assertEqual(plan['route_tasks'][0]['hold_s'], 3)
            self.assertEqual(plan['profile'], 'REPLAY')
            self.assertIs(plan['physical_flight_approval'], False)
            self.assertEqual(plan['coordinate_frame']['z_reference'], 'SYNTHETIC_FLAT_FLOOR')

    def test_xy_retraces_instead_of_diagonal_return(self):
        plan = make_plan('xy', 1)
        self.assertEqual([(t['position_m']['x'], t['position_m']['y']) for t in plan['route_tasks']],
                         [(0, 0), (1, 0), (1, 1), (1, 0), (0, 0)])

    def test_invalid_height_and_case_rejected(self):
        for z in (math.nan, math.inf, -1, 0, 6, True, '1'):
            with self.assertRaises(ValueError):
                make_plan('hover', z)
        with self.assertRaises(ValueError):
            make_plan('arm', 1)

    def test_readback_cannot_grant_flight_and_zero_rc_is_invalid(self):
        report = audit_view({'parameters': {'COM_RC_OVERRIDE': 3, 'EKF2_EV_CTRL': 1},
                             'last_messages': {'/mavros/rc/in': {'channels': [0]*16}}})
        self.assertIs(report['can_fly'], False)
        self.assertIs(report['physical_output_enabled'], False)
        self.assertEqual(report['valid_rc_channels'], 0)
        self.assertEqual(report['anchor_z_m'], .15)


if __name__ == '__main__':
    unittest.main()
