from types import SimpleNamespace as NS
from drone_mission.raw_feedback import feedback
from drone_uwb.integration.clock_readiness import ClockReadiness


def packet(kind='GPS_GLOBAL_ORIGIN', system=1, component=1, **values):
    return NS(get_type=lambda:kind,get_srcSystem=lambda:system,get_srcComponent=lambda:component,
              **dict(dict(latitude=0,longitude=0,time_usec=1000000,
                          coordinate_frame=5,type_mask=0,time_boot_ms=2000,lat_int=123,lon_int=456),**values))


def test_only_fc_origin_is_accepted_and_zero_is_a_valid_projection_reference():
    clock=ClockReadiness()
    result=feedback(packet(),clock,1.,0)
    assert result[0]=='origin' and result[1].latitude_deg==0 and result[1].longitude_deg==0
    assert feedback(packet(system=2),clock,1.,0) is None
    assert feedback(packet(component=197),clock,1.,0) is None


def test_target_feedback_requires_qualified_clock_source_frame_and_fresh_sample():
    c=ClockReadiness(qualified=True,stable_samples=30,last_valid_s=10.,offset_ns=1000000000000)
    p=packet('POSITION_TARGET_GLOBAL_INT')
    now_ns=c.offset_ns+2_100_000_000
    kind,result=feedback(p,c,10.1,now_ns)
    assert kind=='target' and result['stamp_ns']==c.offset_ns+2_000_000_000
    assert feedback(p,c,11.,now_ns) is None
    assert feedback(p,c,10.1,now_ns+1_000_000_000) is None
    assert feedback(p,c,10.1,now_ns-1_000_000_000) is None
    for changes in ({'coordinate_frame':6},{'type_mask':1},{'type_mask':2},{'time_boot_ms':0}):
        assert feedback(packet('POSITION_TARGET_GLOBAL_INT',**changes),c,10.1,now_ns) is None
