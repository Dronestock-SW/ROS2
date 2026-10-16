"""Evidence must retain source time and avoid blocking on slow storage."""
import json
import threading
from types import SimpleNamespace
from drone_mission.recording import AsyncEventRecorder,flight_sample,sensor_sources
from drone_mission.session import Snapshot


def test_slow_write_does_not_block_record_submission(tmp_path):
    entered=threading.Event();release=threading.Event()
    class Slow:
        def __init__(self):self.file=(tmp_path/'events.jsonl').open('w',encoding='utf-8')
        def write(self,text):entered.set();assert release.wait(3);return self.file.write(text)
        def flush(self):self.file.flush()
        def fileno(self):return self.file.fileno()
        def close(self):self.file.close()
    recorder=AsyncEventRecorder(Slow())
    assert recorder.submit(dict(type='sample',source_stamp_ns=123,logged_monotonic_ns=456))
    assert entered.wait(1)
    for i in range(10):assert recorder.submit(dict(type='sample',index=i))
    assert recorder.submitted==11 and recorder.written==0
    release.set();assert recorder.close()
    rows=[json.loads(x) for x in (tmp_path/'events.jsonl').read_text(encoding='utf-8').splitlines()]
    assert [r['record_sequence'] for r in rows]==list(range(1,12))
    assert rows[0]['source_stamp_ns']==123 and rows[0]['logged_monotonic_ns']==456
    assert rows[0]['written_monotonic_ns']>0


def test_sample_keeps_all_age_fields_and_does_not_relabel_source_time():
    stamp=SimpleNamespace(sec=1,nanosec=25)
    msg=SimpleNamespace(header=SimpleNamespace(stamp=stamp,frame_id='uwb_map'))
    sources=sensor_sources({'uwb':(msg,2.)})
    s=Snapshot(uwb_stamp_ns=1_000_000_025,bridge_stamp_ns=99,uwb_age_s=.13,
               bridge_observation_age_s=.48,bridge_age_s=.01,estimator_age_s=.05)
    row=flight_sample(s,phase='TAKING_OFF',target={'xy':[2,2]},feedback=None,
                     observation={'mode':'REPORT_DELAY'},sources=sources,owner='python_executor')
    assert row['input_snapshot']['uwb_stamp_ns']==1_000_000_025
    assert row['input_snapshot']['bridge_observation_age_s']==.48
    assert row['input_snapshot']['estimator_age_s']==.05
    assert sources['uwb']['stamp_ns']==1_000_000_025
    assert sources['uwb']['received_monotonic_ns']==2_000_000_000
    assert row['input_snapshot']['height_age_s'] is None
