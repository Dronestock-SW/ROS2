import json
from pathlib import Path
import pytest

from drone_uwb.integration.manual_analysis import analyze, main


def row(topic, data, t, kind='std_msgs/msg/String'):
    if kind=='std_msgs/msg/String': data={'data':json.dumps(data)}
    return dict(topic=topic, data=data, received_monotonic_ns=int(t*1e9), type=kind)


def test_fresh_armed_counts_and_flow_of_original_clock_envelopes():
    events=[row('/mavros/state',dict(connected=True,armed=True,mode='POSCTL'),1,'mavros_msgs/msg/State'),
            row('/uwb/btf_decision',dict(reason='clock_unsynced'),1.1),
            row('/uwb/btf_decision',dict(reason='clock_unsynced'),3),
            row('/mavros/downward_0',dict(range='NaN',min_range=.1,max_range=6),3.1,'sensor_msgs/msg/Range'),
            row('/mavros/downward_0',dict(range=.85,min_range=.1,max_range=6),3.2,'sensor_msgs/msg/Range')]
    result=analyze(events,tag_id='6')
    assert result['recorded_btf_reasons']['armed']=={'clock_unsynced':1}
    assert result['recorded_btf_reasons']['unknown']=={'clock_unsynced':1}
    assert result['max_valid_downward_range_m']==.85
    assert not result['flight_authorized'] and not result['timing_confirmed']


def test_cli_rejects_truncated_capture_and_never_overwrites_evidence(tmp_path):
    (tmp_path/'manifest.json').write_text(json.dumps(dict(tag_id='6',boot_id='x',started={})))
    events=tmp_path/'events.jsonl'; events.write_text('{"topic":')
    before=events.read_bytes()
    with pytest.raises(SystemExit): main([str(tmp_path),'--output',str(events)])
    with pytest.raises(json.JSONDecodeError): main([str(tmp_path),'--output',str(tmp_path/'analysis.json')])
    assert events.read_bytes()==before and not (tmp_path/'analysis.json').exists()
