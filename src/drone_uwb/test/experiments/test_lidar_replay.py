import json
import numpy as np
import pytest

from drone_uwb.processing.experiments.lidar_replay import GyroSeries, file_sha256, json_safe, replay
from drone_uwb.processing.experiments.lidar_synthetic import cast_scan


def test_gyro_integrates_rate_and_interpolates_without_extrapolation():
    gyro = GyroSeries([0., .01, .02], [[0,0,0], [0,0,1], [0,0,2]], np.eye(3))
    assert gyro.at(.005)[1] == pytest.approx(.00125)
    assert gyro.at(.02)[1] == pytest.approx(.02)
    assert gyro.at(-.001) is None
    assert gyro.at(.021) is None
    assert gyro.at(float('nan')) is None


def test_gyro_gap_splits_epochs_and_does_not_integrate_gap():
    gyro = GyroSeries([0., .01, .1, .11], [[0,0,1]] * 4, np.eye(3))
    assert gyro.at(.05) is None
    assert gyro.at(.005)[0] != gyro.at(.105)[0]
    assert gyro.at(.11)[1] == pytest.approx(.02)


def test_gyro_invalid_mount_and_clock_are_rejected():
    with pytest.raises(ValueError, match='proper rotation'):
        GyroSeries([0., .01], [[0,0,1]] * 2, np.diag([1,1,-1]))
    with pytest.raises(ValueError, match='timestamps must increase'):
        GyroSeries([.01, 0.], [[0,0,1]] * 2, np.eye(3))


def test_hash_mismatch_produces_no_output(tmp_path):
    raw = tmp_path / 'events.jsonl'
    raw.write_text('{}\n', encoding='utf-8')
    with pytest.raises(ValueError, match='SHA256 mismatch'):
        replay(raw, '0'*64, tmp_path / 'result')
    assert not (tmp_path / 'result').exists()


def test_capture_schema_and_lidar_only_scope(tmp_path):
    raw = tmp_path / 'events.jsonl'
    ranges = cast_scan([0,0,0], np.random.default_rng(42))
    scan = dict(ranges=ranges, angle_min=-np.pi, angle_increment=2*np.pi/430,
                range_min=.03, range_max=12., scan_time=.1, time_increment=.1/430)
    rows = [dict(topic='/scan', header_ns=1_000_000_000 + i*100_000_000, data=scan) for i in range(3)]
    raw.write_text(''.join(json.dumps(json_safe(r), allow_nan=False)+'\n' for r in rows), encoding='utf-8')
    result = replay(raw, file_sha256(raw), tmp_path / 'result')
    assert result['accepted_displacements'] == 2
    assert result['original_minus267_reproduced'] is False
    assert result['imu_gate'] == 'NOT_EVALUATED_NO_MOUNT_ROTATION'
    assert file_sha256(raw) == result['raw_sha256']
