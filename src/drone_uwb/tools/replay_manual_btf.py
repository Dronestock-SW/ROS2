"""File-only MeasuredBtf replay in collector order, not original callback order.

No ROS imports, sockets, FC pose feedback or flight commands. Clock/height
qualification mirrors the ROS adapter; collector order is only an approximation.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

from drone_uwb.integration.clock_readiness import ClockReadiness
from drone_uwb.processing.measured_btf import MeasuredBtf, ground_xy_without_height


def replay(capture, output):
    manifest=json.loads((capture/'manifest.json').read_text(encoding='utf-8'))
    evidence=next(r for r in manifest['config_evidence'] if r['name']=='field-btf.json')
    assert hashlib.sha256(evidence['content_utf8'].encode()).hexdigest()==evidence['sha256']
    config=json.loads(evidence['content_utf8'])
    assert config['external_output_allowed'] is False
    processor=MeasuredBtf(config); clock=ClockReadiness()
    state={}; landed=None; state_at=landed_at=velocity_at=float('-inf')
    velocity_ok=False; landed_stamp=0; counts=Counter(); last=0
    with (capture/'events.jsonl').open(encoding='utf-8') as src, output.open('w',encoding='utf-8') as dst:
        for line in src:
            row=json.loads(line); topic=row['topic']; d=row['data']
            mono,ros=row['received_monotonic_ns'],row['received_ros_ns']
            if mono<last: raise ValueError('receipt_clock_reversed')
            last=mono; now=mono/1e9; stamp=row.get('header_ns') or 0
            if topic=='/mavros/state':
                state=d; fresh=0<=ros-stamp<=1_500_000_000
                state_at=now if fresh else float('-inf')
                if not fresh or not d['connected']:clock.reset()
            elif topic=='/mavros/extended_state':
                landed=d['landed_state'];landed_stamp=stamp
                landed_at=now if 0<=ros-stamp<=1_500_000_000 else float('-inf')
            elif topic=='/mavros/local_position/odom':
                v=[float(d['twist']['twist']['linear'][k]) for k in ('x','y','z')]
                velocity_ok=(all(math.isfinite(x) for x in v) and abs(v[2])<=.05
                    and math.sqrt(sum(x*x for x in v))<=.1 and 0<=ros-stamp<=200_000_000)
                velocity_at=now
            elif topic=='/mavros/timesync_status':
                _,changed=clock.observe(now=now,remote_ns=d['remote_timestamp_ns'],
                    offset_ns=d['estimated_offset_ns'],rtt_ms=d['round_trip_time_ms'])
                if changed:
                    for samples in processor.height.samples.values():samples.clear()
                    processor.reset_models()
            elif topic in (config['tof_topic'],config['imu_topic']):
                valid=clock.ready(now) and 0<=ros-stamp<=500_000_000
                if topic==config['tof_topic']:
                    r,lo,hi=(float(d[k]) for k in ('range','min_range','max_range'))
                    valid &= all(math.isfinite(x) for x in (r,lo,hi)) and 0<lo<=r<=hi
                    processor.height.add('tof',dict(stamp_ns=stamp,range_m=r,valid=valid))
                else:
                    q=[float(d['orientation'][k]) for k in ('w','x','y','z')]
                    valid &= all(math.isfinite(x) for x in q) and abs(sum(x*x for x in q)-1)<.02
                    valid &= d['orientation_covariance'][0]>=0 and d['header']['frame_id']=='base_link'
                    processor.height.add('imu',dict(stamp_ns=stamp,quaternion_wxyz=q,valid=valid))
            elif topic=='/uwb/received':
                event=json.loads(d['data']);age=ros-event['host_received_ros_ns']
                if not 0<=age<=150_000_000:
                    processor.reset() if age<0 else processor.reject_queued_input()
                    counts['receiver_queue_expired']+=1
                    continue
                ready=clock.ready(now)
                if not ready:
                    for samples in processor.height.samples.values():samples.clear()
                ground=(ready and state.get('connected') is True and landed==1
                    and now-state_at<=1.5 and now-landed_at<=1.5 and velocity_ok and now-velocity_at<=.2)
                processor.height.ground_reference=(dict(height_m=config['ground_antenna_height_m'],
                    selection_stamp_ns=ros,ground_state_stamp_ns=landed_stamp)
                    if ground and config.get('ground_antenna_height_m') is not None else None)
                result=processor.process(event)
                if result['reason'].startswith('sideband_') or result['reason'] in ('status','awaiting_tdma'):continue
                height_ok=result.get('xyz_m') is not None or ground_xy_without_height(result,
                    enabled=config.get('allow_ground_xy_without_tof',False),connected=state.get('connected'),
                    landed=landed,state_age_s=now-state_at,landed_age_s=now-landed_at)
                publish=bool(result['ok'] and height_ok and 0<=ros-result['stamp_ns']<=config['max_output_age_s']*1e9)
                airborne=(state.get('armed') is True and state.get('connected') is True and landed==2
                    and 0<=now-state_at<=1.5 and 0<=now-landed_at<=1.5)
                result.update(received_monotonic_ns=mono,airborne=airborne,replay_publish=publish)
                result['replay_guard_prior']={'xy_m':processor.raw_guard.prior_xy,
                    'stamp_ns':processor.raw_guard.prior_stamp}
                dst.write(json.dumps(result,allow_nan=False)+'\n')
                if airborne:counts[result['reason']]+=1;counts['published']+=int(publish)
    return dict(capture=capture.name,counts=dict(counts),scope='collector-order replay; no physical validation')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture',type=Path);p.add_argument('--output',required=True,type=Path)
    args=p.parse_args();print(json.dumps(replay(args.capture,args.output)))


if __name__=='__main__':main()
