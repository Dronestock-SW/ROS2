from unittest.mock import patch
import pytest
from drone_platform_link.telemetry import Observations


@pytest.mark.parametrize('source',['uwb_xy','btf_xy'])
def test_xy_sources_never_export_pose_z_placeholder(source):
    o=Observations(source)
    assert o.receive_pose(1,2,'uwb_map',1_000_000_000,now_ns=1_100_000_000,z=0)
    f=o.fields()
    assert f['pose_source']==source and f['fix']
    assert f['current_z_m'] is None and f['z_source'] is None
    assert not f['xyz_valid']
    assert f['source_age_ms']>=100


def test_original_age_does_not_restart_at_receipt_or_duplicate():
    with patch('drone_platform_link.telemetry.time.monotonic',return_value=10):
        o=Observations('btf_xy')
        assert o.receive_pose(1,2,'uwb_map',1_000_000_000,now_ns=1_400_000_000)
        assert not o.receive_pose(9,9,'uwb_map',1_000_000_000,now_ns=1_400_000_000)
    with patch('drone_platform_link.telemetry.time.monotonic',return_value=10.15):
        f=o.fields()
    assert f['source_age_ms']==550
    assert not f['fix'] and f['x'] is None


def test_px4_source_retains_local_frame_and_same_stamp_xyz():
    o=Observations('px4_local')
    assert o.receive_pose(1,2,'map',1_000_000_000,now_ns=1_000_000_000,z=3)
    f=o.fields()
    assert f['coordinate_frame']=='px4_local_enu' and f['position_reference']=='fc'
    assert f['position_kind']=='state_estimate' and f['xyz_valid']
    assert f['current_z_m']==3 and f['source_stamp_ns']==1_000_000_000
    assert not o.receive_pose(1,2,'uwb_map',2_000_000_000,now_ns=2_000_000_000,z=3)
    assert not o.fields()['fix']
