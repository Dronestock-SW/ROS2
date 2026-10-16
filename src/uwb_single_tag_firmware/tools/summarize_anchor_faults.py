"""Describe observed single-anchor losses; never label SWD halt as power loss."""
import argparse
from collections import Counter
import json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('events',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    raw=[];status=[]
    with args.events.open(encoding='utf-8') as stream:
        for line in stream:
            row=json.loads(line)
            if row['topic']!='/uwb/raw':continue
            value=row['value']
            item=(row['capture_monotonic_ns'],value)
            if value.get('type')=='uwb_raw_cycle':raw.append(item)
            if value.get('type')=='uwb_raw_status':status.append(item)
    if not raw:raise RuntimeError('No RAW')
    first=raw[0][0];windows=[]
    for anchor in range(4):
        missing=[(t,m) for t,m in raw if not m['valid_mask']&(1<<anchor)]
        if not missing:
            windows.append(dict(anchor='A'+str(anchor+1),observed_loss=False));continue
        groups=[]
        for item in missing:
            if not groups or item[0]-groups[-1][-1][0]>200_000_000:groups.append([])
            groups[-1].append(item)
        sustained=max(groups,key=len)
        start,end=sustained[0][0],sustained[-1][0]
        inside=[(t,m) for t,m in raw if start<=t<=end]
        hb=[m for t,m in status if start<=t<=end]
        target=15^(1<<anchor)
        recovered=next((t for t,m in raw if t>end and m['valid_mask']==15),None)
        windows.append(dict(anchor='A'+str(anchor+1),observed_loss=True,
            first_loss_s=(start-first)/1e9,last_loss_s=(end-first)/1e9,
            missing_outside_sustained=len(missing)-len(sustained),
            raw_count=len(inside),masks=dict(Counter(m['valid_mask'] for _,m in inside)),
            other_three_valid_ratio=sum(m['valid_mask']==target for _,m in inside)/len(inside),
            max_raw_interval_ms=max(((b[0]-a[0])/1e6 for a,b in zip(inside,inside[1:])),default=None),
            lora_done_first=hb[0]['lora_tx_done'] if hb else None,
            lora_done_last=hb[-1]['lora_tx_done'] if hb else None,
            recovered_all_four_s=(recovered-first)/1e9 if recovered else None))
    result=dict(test='physical_anchor_MCU_halt_via_SWD',power_removed=False,
        raw_count=len(raw),valid_masks=dict(Counter(m['valid_mask'] for _,m in raw)),
        windows=windows,first_raw=raw[0][1],last_raw=raw[-1][1],
        first_heartbeat=status[0][1] if status else None,last_heartbeat=status[-1][1] if status else None)
    args.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('test','power_removed','raw_count','valid_masks','windows')}))


if __name__=='__main__':main()
