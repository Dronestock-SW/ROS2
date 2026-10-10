"""Check recorded real B_TF decisions against their ordered measured inputs."""
import argparse
import json
from pathlib import Path
from drone_uwb.processing.measured_btf import MeasuredBtf
from drone_uwb.processing.observation_guard import ObservationGuard


def apply_recorded_reset(processor, row):
    if row['type']=='uwb_rejected':
        if row.get('reset_scope')=='queued_models':
            processor.reject_queued_input()
        elif row.get('reset_scope')=='all':
            processor.reset()
        else:
            raise ValueError('legacy rejection lacks reset scope; exact replay is not provable')
    elif row['type']=='ground_session_reset' and row['accepted']:
        # Replay an accepted recorded event; this function has no live ROS path.
        processor.reset_models()
        processor.raw_guard=ObservationGuard()
        processor.observation_session=row['observation_session']


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('capture',type=Path,help='BTF directory containing config/inputs/decisions')
    args=parser.parse_args()
    processor=MeasuredBtf(json.loads((args.capture/'config.json').read_text(encoding='utf-8')))
    expected=iter(json.loads(line) for line in (args.capture/'decisions.jsonl').read_text(encoding='utf-8').splitlines())
    count=0
    last_offset=None
    for line in (args.capture/'inputs.jsonl').read_text(encoding='utf-8').splitlines():
        row=json.loads(line)
        if row['type'] in ('tof','imu'):
            processor.height.add(row['type'],row)
        elif row['type']=='timesync':
            offset=row['estimated_offset_ns']
            if last_offset is not None and abs(offset-last_offset)>5_000_000:
                for target in processor.height.samples.values():target.clear()
                processor.reset_models()
            last_offset=offset
        elif row['type'] in ('uwb_rejected','ground_session_reset'):
            apply_recorded_reset(processor,row)
        elif row['type']=='uwb':
            if not row['clock_sync_ready']:
                for target in processor.height.samples.values():target.clear()
            actual=processor.process(row['event'])
            recorded=next(expected)
            for field in ('reason','ok','xy_m','height_m','time_us','sample_time_us'):
                assert actual.get(field)==recorded.get(field),(count,field,actual.get(field),recorded.get(field))
            count+=1
    assert next(expected,None) is None,'extra recorded decisions'
    print(json.dumps({'matched_decisions':count,'accuracy_validation':False,'scope':'ordered-input replay only'},indent=2))


if __name__=='__main__':
    main()
