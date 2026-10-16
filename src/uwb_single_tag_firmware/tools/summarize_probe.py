"""Summarize a direct UART probe without hiding reset or malformed-input gaps."""
import argparse
from collections import Counter
import json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('events',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    rows=[json.loads(line) for line in args.events.read_text(encoding='utf-8').splitlines()]
    raw=[r for r in rows if r.get('message',{}).get('type')=='uwb_raw_cycle' and r.get('direction')!='host_to_tag']
    heartbeats=[r['message'] for r in rows if r.get('message',{}).get('type')=='uwb_raw_status']
    acks=Counter((r['message']['accepted'],r['message']['reason']) for r in rows
                 if r.get('message',{}).get('type')=='jetson_pose_ack')
    stages=[r for r in rows if 'probe_stage' in r]
    windows=[]
    for i,stage in enumerate(stages):
        start=stage['host_monotonic_ns']
        stop=stages[i+1]['host_monotonic_ns'] if i+1<len(stages) else 2**63
        subset=[r for r in raw if start<=r['host_received_monotonic_ns']<stop]
        gaps=[(b['host_received_monotonic_ns']-a['host_received_monotonic_ns'])/1e6 for a,b in zip(subset,subset[1:])]
        windows.append(dict(stage=stage['probe_stage'],raw_count=len(subset),
            valid_masks=dict(Counter(r['message']['valid_mask'] for r in subset)),
            max_interval_ms=max(gaps,default=None),
            tx_dropped_first=subset[0]['message']['tx_dropped'] if subset else None,
            tx_dropped_last=subset[-1]['message']['tx_dropped'] if subset else None))
    gaps=[]
    for a,b in zip(raw,raw[1:]):
        ma,mb=a['message'],b['message']
        elapsed=(b['host_received_monotonic_ns']-a['host_received_monotonic_ns'])/1e6
        delta=(mb['seq']-ma['seq'])&0xffffffff
        if elapsed>=100 or delta!=1:
            gaps.append(dict(host_interval_ms=elapsed,seq_before=ma['seq'],seq_after=mb['seq'],
                source_interval_us=mb['cycle_start_us']-ma['cycle_start_us'],
                session_changed=(ma['boot_id'],ma['session'])!=(mb['boot_id'],mb['session']),
                tx_dropped_before=ma['tx_dropped'],tx_dropped_after=mb['tx_dropped'],
                host_monotonic_ns=b['host_received_monotonic_ns']))
    result=dict(raw_received=len(raw),stages=stages,windows=windows,gaps=gaps,
        pose_acks=[dict(accepted=a,reason=r,count=c) for (a,r),c in acks.items()],
        first_heartbeat=heartbeats[0] if heartbeats else None,last_heartbeat=heartbeats[-1] if heartbeats else None,
        lora_fix_true_heartbeats=sum(r.get('lora_last_tx_fix') is True for r in heartbeats))
    args.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('raw_received','windows','gaps','pose_acks','lora_fix_true_heartbeats')}))


if __name__=='__main__':main()
