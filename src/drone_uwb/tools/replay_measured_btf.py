"""Check recorded real B_TF decisions against their ordered measured inputs."""
import argparse
import json
from pathlib import Path
from drone_uwb.processing.measured_btf import MeasuredBtf


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
        elif row['type']=='uwb_rejected':
            processor.reset()
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
