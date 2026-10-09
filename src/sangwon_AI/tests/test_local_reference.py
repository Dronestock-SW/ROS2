"""Origin assignment cannot turn invalid ground state into valid position."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('reference', Path(__file__).resolve().parents[1]/'ops/px4_local_reference.py')
ref = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ref)

LOCAL = '''ref_timestamp: 0
ref_lat: nan
ref_lon: nan
ref_alt: nan
x: 0.1
y: 0.2
z: -0.3
vx: 0.01
vy: 0.01
vz: 0.01
heading: 1.5
xy_valid: True
z_valid: True
v_xy_valid: True
v_z_valid: True
heading_good_for_control: True
'''
ARMED = '''armed: False
prearmed: False
termination: False
in_esc_calibration_mode: False
'''
FLAGS = '''manual_control_signal_lost: False
local_position_invalid: False
local_altitude_invalid: False
'''


class ReferenceTest(unittest.TestCase):
    def test_preserve_existing_vertical_frame(self):
        before = LOCAL+'dist_bottom: 0.39\ndist_bottom_valid: True\nz_reset_counter: 2\n'
        after = before.replace('z: -0.3\n', 'z: 63.34\n').replace('ref_alt: nan', 'ref_alt: 58').replace('z_reset_counter: 2','z_reset_counter: 3')+'delta_z: -63.64\n'
        self.assertAlmostEqual(ref.original_vertical_datum(before, after), -5.64)
        for bad in (after.replace('dist_bottom: 0.39','dist_bottom: 1.1'),
                    after.replace('z_reset_counter: 3','z_reset_counter: 4'),
                    after.replace('delta_z: -63.64','delta_z: 63.64')):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                ref.original_vertical_datum(before, bad)

    def test_valid_ground(self):
        ref.validate_ground(LOCAL, ARMED, FLAGS)

    def test_active_or_invalid_ground(self):
        for name in ('armed', 'prearmed', 'termination', 'in_esc_calibration_mode'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                ref.validate_ground(LOCAL, ARMED.replace(name+': False', name+': True'), FLAGS)
        for name in ('xy_valid', 'z_valid', 'v_xy_valid', 'v_z_valid', 'heading_good_for_control'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                ref.validate_ground(LOCAL.replace(name+': True', name+': False'), ARMED, FLAGS)

    def test_existing_origin_preserved(self):
        for old, new in (('ref_timestamp: 0', 'ref_timestamp: 42'), ('ref_lat: nan', 'ref_lat: 36.3')):
            with self.subTest(new=new), self.assertRaises(ValueError):
                ref.validate_ground(LOCAL.replace(old, new), ARMED, FLAGS)

    def test_stale_ambiguous_or_bad_values(self):
        for text in (LOCAL.replace('vx: 0.01', 'vx: 0.5'), LOCAL.replace('x: 0.1', 'x: nan'),
                     LOCAL+'xy_valid: True\n', LOCAL.replace('xy_valid: True\n', '')):
            with self.subTest(text=text), self.assertRaises(ValueError):
                ref.validate_ground(text, ARMED, FLAGS)

    def test_rc_and_position_failsafe(self):
        for name in ('manual_control_signal_lost', 'local_position_invalid', 'local_altitude_invalid'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                ref.validate_ground(LOCAL, ARMED, FLAGS.replace(name+': False', name+': True'))

    def test_reference_requires_provenance(self):
        valid = dict(latitude=36.3, longitude=127.4, altitude_amsl_m=58.,
                     quality='regional_approximation', site='synthetic test', sources=['fixture'])
        self.assertEqual(ref.validate_reference(valid), valid)
        for key, value in (('latitude', 91), ('longitude', float('nan')), ('altitude_amsl_m', True),
                           ('sources', []), ('quality', 'verified'), ('site', '')):
            with self.subTest(key=key), self.assertRaises(ValueError):
                ref.validate_reference(dict(valid, **{key: value}))


if __name__ == '__main__':
    unittest.main()
