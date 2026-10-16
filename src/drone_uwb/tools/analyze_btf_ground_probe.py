"""Summarize sparse physical FC readbacks; do not infer per-packet fusion."""
import argparse
import json
from pathlib import Path
import re


def analyze(root):
    sent=[r for l in (root/'sent.jsonl').open(encoding='utf-8') if not (r:=json.loads(l)).get('expired')]
    answers=json.loads((root/'readback.json').read_text(encoding='utf-8'))
    samples=[];fusions=[];flags=[]
    for answer in answers:
        text=answer['response'];phase=answer['phase']
        if 'vehicle_visual_odometry' in answer['command'] and phase=='inject':
            t=re.search(r'timestamp_sample: (\d+)',text)
            xy=re.search(r'position: \[([^]]+)\]',text)
            if not t or not xy:continue
            x,y,*_=map(float,xy[1].split(','))
            match=min(sent,key=lambda r:abs(r['x']-x)+abs(r['y']-y))
            delta=max(abs(match['x']-x),abs(match['y']-y))
            # Five-decimal console output truncates coordinates.
            samples.append(dict(elapsed_s=answer['elapsed_s'],xy_max_delta_m=delta,
                matched=delta<=1.1e-5,fc_sample_minus_translated_capture_us=
                int(t[1])-(match['capture_us']-match['fc_estimated_offset_ns']/1000)))
        elif 'estimator_aid_src_ev_pos' in answer['command']:
            fusions.append(dict(elapsed_s=answer['elapsed_s'],phase=phase,
                time_last_fuse_us=list(map(int,re.findall(r'time_last_fuse: (\d+)',text))),
                fused=re.findall(r'\bfused: (True|False)',text),
                innovation_rejected=re.findall(r'innovation_rejected: (True|False)',text)))
        elif 'estimator_status_flags' in answer['command']:
            flags.append(dict(elapsed_s=answer['elapsed_s'],phase=phase,
                ev_pos=re.findall(r'cs_ev_pos: (True|False)',text)))
    return dict(scope=__doc__,result=json.loads((root/'result.json').read_text()),
        maximum_sender_gap_s=max(((b['send_ros_ns']-a['send_ros_ns'])/1e9 for a,b in zip(sent,sent[1:])),default=None),
        visual_samples=samples,aid_samples=fusions,flag_samples=flags,
        all_packets_received_or_fused_verified=False,physical_flight_verified=False)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--probe',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    result=analyze(args.probe)
    args.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result))
