"""Offline checks for the ground-only firmware setup tool."""
import importlib.util
from pathlib import Path

import pytest


TOOLS = Path(__file__).resolve().parents[2] / 'tools'
SPEC = importlib.util.spec_from_file_location('lidar2d_fc_setup', TOOLS / 'configure_lidar2d_fc.py')
setup = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(setup)


def number(value):
    return {'type': 2, 'integer_value': value} if isinstance(value, int) else {'type': 3, 'double_value': value}


def test_only_two_lidar_settings_are_written():
    assert setup.TARGET == {'EKF2_LID2_AGE': 180., 'EKF2_LID2_EN': 1}


@pytest.mark.parametrize('name', ['COM_RC_OVERRIDE', 'MPC_TKO_SPEED', 'EKF2_EV_CTRL', 'CAL_MAG0_XOFF'])
def test_safety_sensor_and_calibration_changes_are_reported(name):
    assert name in setup.differences({name: number(1)}, {name: number(2)})


def test_missing_parameter_is_not_treated_as_preserved():
    assert setup.differences({'EKF2_EV_CTRL': number(1)}, {}) == {
        'EKF2_EV_CTRL': {'before': 1, 'after': None}}


def test_metadata_counters_and_intended_changes_are_separate():
    before = {k: number(0) for k in setup.IGNORED}
    after = {k: number(1) for k in setup.IGNORED}
    assert setup.differences(before, after) == {}


def test_hardware_patch_preserves_crsf_and_exports_version_marker():
    patch = (TOOLS / 'px4_lidar2d_hardware.patch').read_text(encoding='utf-8')
    assert '+CONFIG_DRIVERS_RC_CRSF_RC=y' in patch
    assert '(ParamInt<px4::params::EKF2_LID2_VER>) _param_ekf2_lid2_ver' in patch
