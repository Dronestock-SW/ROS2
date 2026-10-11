import json

import pytest

from drone_uwb.integration.capture_transport_audit import audit


def row(mono, ros, header, topic='sensor'):
    return json.dumps(dict(topic=topic,received_monotonic_ns=mono,
                           received_ros_ns=ros,header_ns=header))


def test_separates_backlog_signature_from_source_progress_and_clock_step():
    offset = 1_791_000_000_000_000_000
    before = 1_000_000_000
    after = 19_000_000_000
    result = audit([
        row(before,offset+before,offset+before,t) for t in ('queued','current')
    ]+[
        row(after,offset+after,offset+before+1_000_000_000,'queued'),
        row(after,offset+after,offset+after,'current')])
    assert result['receipt_clock_offset_span_ns']==0
    queued=result['topics']['queued']['largest_gaps'][0]
    current=result['topics']['current']['largest_gaps'][0]
    assert queued['receipt_gap_ns']==current['receipt_gap_ns']==18_000_000_000
    assert queued['header_age_after_ns']==17_000_000_000
    assert current['header_age_after_ns']==0
    assert not result['timing_confirmed'] and not result['flight_authorized']
    stepped=audit([row(before,offset+before,offset+before),row(after,offset+after+500,offset+after)])
    assert stepped['receipt_clock_offset_span_ns']==500
    assert stepped['topics']['sensor']['largest_gaps'][0]['receipt_clock_offset_change_ns']==500


def test_nanosecond_precision_future_and_regressions_are_not_silenced():
    t=1_791_000_000_000_000_000
    result=audit([row(t,t,t+1),row(t-1,t-1,t-2),row(t+2,t+2,None)])
    s=result['topics']['sensor']
    assert s['future_header_count']==1 and s['min_header_age_ns']==-1
    assert s['receipt_clock_regressions']==s['header_clock_regressions']==1
    assert s['missing_header_count']==1


def test_bounded_top_gaps_and_rejects_incomplete_or_invalid_rows():
    result=audit([row(n*n,n*n,None) for n in range(100)],top=2)
    assert [x['receipt_gap_ns'] for x in result['topics']['sensor']['largest_gaps']]==[197,195]
    for malformed in ('{',row(1.5,2,None),row(True,2,None),row(1,2,'3')):
        with pytest.raises(ValueError,match='invalid_capture_row:2'):
            audit([row(1,2,1),malformed])
    assert audit([])['rows']==0
