"""Offline, explicit-window audit of recorded observations, not position truth.

Uses host monotonic time, preserving sensor clocks in the exported events.
No ROS connection, parameter changes, pose corrections or publication.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def stats(values):
    a = np.asarray(values, float)
    a = a[np.isfinite(a)]
    return (dict(n=len(a), min=float(a.min()), median=float(np.median(a)),
                 p05=float(np.quantile(a, .05)), p95=float(np.quantile(a, .95)),
                 max=float(a.max())) if len(a) else dict(n=0))


def motion(rows, start, end):
    if not rows:
        return dict(n=0)
    a = np.asarray(rows, float)
    a = a[np.isfinite(a).all(axis=1)]
    if not len(a):
        return dict(n=0, nonfinite_rows=len(rows))
    if not (np.any(a[:,0] < start+5) and np.any(a[:,0] > end-5)):
        return dict(n=len(a), endpoint_coverage=False)
    first = np.median(a[a[:, 0] < start+5, 1:], axis=0)
    last = np.median(a[a[:, 0] > end-5, 1:], axis=0)
    steps = np.linalg.norm(np.diff(a[:, 1:], axis=0), axis=1)
    return dict(n=len(a), nonfinite_rows=len(rows)-len(a), span_m=np.ptp(a[:, 1:], axis=0).tolist(),
                first_5s_median=first.tolist(), last_5s_median=last.tolist(),
                endpoint_delta_m=(last-first).tolist(),
                endpoint_norm_m=float(np.linalg.norm(last-first)),
                adjacent_step_m=stats(steps),
                max_receipt_gap_s=float(np.diff(a[:, 0]).max()) if len(a)>1 else None)


def angles(q):
    x,y,z,w = (q[k] for k in ('x','y','z','w'))
    return [math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
            math.asin(np.clip(2*(w*y-z*x),-1,1)),
            math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))]


def audit(capture, start, end, output):
    manifest = json.loads((capture/'manifest.json').read_text(encoding='utf-8'))
    evidence = next(c for c in manifest['config_evidence'] if c['name']=='field-btf.json')
    if hashlib.sha256(evidence['content_utf8'].encode()).hexdigest()!=evidence['sha256']:
        raise ValueError('Recorded configuration hash mismatch')
    config = json.loads(evidence['content_utf8'])
    anchors = np.asarray(config['anchors_xyz_m'], float)
    rows = defaultdict(list); topics = defaultdict(list); decisions=[]; raw=[]
    selected_events=[]; physical=[]; scans=[]; rc_incomplete=0
    for line in (capture/'events.jsonl').open(encoding='utf-8'):
        e=json.loads(line); t=e['received_monotonic_ns']/1e9; topic=e['topic']; d=e['data']
        if topic in ('/mavros/state','/mavros/extended_state'):
            state = ({k:d[k] for k in ('connected','armed','mode')} if topic.endswith('/state')
                     else {'landed_state':d['landed_state']})
            rows[topic].append([t,state])
        if not start <= t <= end:continue
        topics[topic].append(t)
        if topic.startswith('/uwb/') and isinstance(d.get('data'),str):
            try:d=json.loads(d['data'])
            except ValueError:continue
        if topic=='/uwb/received':
            m=d.get('message',{})
            if m.get('type')=='uwb_raw_cycle':raw.append((t,m))
        elif topic=='/uwb/btf_decision':
            decisions.append((t,d))
            b=d.get('models',{}).get('B',{}); hs=d.get('sample_height_m')
            if (b.get('ok') and d.get('height_source')=='measured_tof_imu'
                    and hs and all(v is not None for v in hs)):
                dt=(np.array(d['sample_time_us'])-d['time_us'])/1e6
                xy=np.asarray(b['xy_m'])+dt[:,None]*np.asarray(b['velocity_m_s'])
                xyz=np.column_stack([xy,hs]); residual=np.linalg.norm(xyz-anchors,axis=1)-d['cal_slant_m']
                physical.append([t,float(np.sqrt(np.mean(residual**2))),*residual,
                                 b['fit']['rms_m'],d['seq'],int(d.get('published') is True and
                                 d.get('models',{}).get('B_TF',{}).get('source')=='unchanged_B4')])
            if d.get('height_source')=='measured_tof_imu' and d.get('height_m') is not None:
                rows['antenna_height'].append([t,d['height_m']])
        elif topic=='/uwb/btf_pose':
            p=d['pose']['pose']['position'];rows['uwb'].append([t,p['x'],p['y']])
        elif topic=='/mavros/local_position/odom':
            p=d['pose']['pose']['position'];rows['odom'].append([t,p['x'],p['y'],p['z']])
        elif topic=='/mavros/imu/data':rows['attitude'].append([t,*angles(d['orientation'])])
        elif topic=='/mavros/downward_0':
            value=float(d['range']);rows['tof'].append([t,value,int(d['min_range']<=value<=d['max_range'])])
        elif topic=='/mavros/rc/in':
            if len(d['channels'])>=4:rows['rc'].append([t,*d['channels'][:4]])
            else:rc_incomplete+=1
        elif topic=='/mavros/timesync_status':
            rows['timesync'].append([t,d['remote_timestamp_ns']/1e9,d['round_trip_time_ms']])
        elif topic=='/scan':
            scan=np.asarray(d['ranges'],float);valid=np.isfinite(scan)&(scan>=d['range_min'])&(scan<=d['range_max'])
            rows['scan_health'].append([t,float(valid.mean()),d['scan_time'],d['time_increment']])
            scans.append(dict(t=t,**d))
        if topic in ('/uwb/btf_decision','/uwb/received'):selected_events.append(dict(t=t,topic=topic,data=d))
    report=dict(capture=capture.name,boot_id=manifest['boot_id'],source_revision=manifest['source_revision'],
                window_monotonic_s=[start,end],independent_position_truth=False,
                topics={k:dict(n=len(ts),hz=len(ts)/(end-start),max_gap_s=max(np.diff(ts),default=0.)) for k,ts in topics.items()},
                uwb_xy=motion(rows['uwb'],start,end),px4_enu=motion(rows['odom'],start,end),
                reasons=dict(Counter(d.get('reason') for _,d in decisions)),
                sources=dict(Counter(d.get('models',{}).get('B_TF',{}).get('source') for _,d in decisions)),
                decisions=len(decisions),published=sum(d.get('published') is True for _,d in decisions),
                incomplete_rc_samples=rc_incomplete,
                raw_cycles=len(raw),raw_missing_by_anchor=[sum(not(m.get('valid_mask',0)&(1<<i)) for _,m in raw) for i in range(4)],
                raw_failure_by_anchor=[dict(Counter(m['failure'][i] for _,m in raw)) for i in range(4)],
                height_selection=dict(Counter(h['reason'] for _,d in decisions for h in d.get('height_selection',[]))))
    for key in ('/mavros/state','/mavros/extended_state'):
        changes=[];last=None
        for t,d in rows[key]:
            if d!=last:changes.append(dict(t=t,**d));last=d
        report[key]=changes
    for key in ('tof','antenna_height','scan_health'):
        a=np.asarray(rows[key],float)
        report[key]=[stats(a[:,i]) for i in range(1,a.shape[1])] if len(a) else []
    a=np.asarray(rows['attitude'],float)
    if len(a):
        a[:,1:]=np.rad2deg(np.unwrap(a[:,1:],axis=0))
        report['attitude_deg']=dict(axis_stats=[stats(a[:,i]) for i in range(1,4)],
            endpoint_delta=(np.median(a[a[:,0]>end-5,1:],axis=0)-np.median(a[a[:,0]<start+5,1:],axis=0)).tolist())
    p=np.asarray(physical,float)
    if len(p):
        report['physical_B4']=dict(n=len(p),rms_m=stats(p[:,1]),over_6cm=int((p[:,1]>=.06).sum()),
                                  residual_by_anchor=[stats(p[:,i]) for i in range(2,6)],fit_rms_m=stats(p[:,6]))
        accepted=p[p[:,8]==1]
        report['physical_published_B4']=dict(n=len(accepted),rms_m=stats(accepted[:,1]),
            over_6cm=int((accepted[:,1]>=.06).sum()),
            residual_by_anchor=[stats(accepted[:,i]) for i in range(2,6)],fit_rms_m=stats(accepted[:,6]))
    if raw:
        ranges=np.array([[t,*m['raw_slant_m']] for t,m in raw],float)
        report['raw_ranges']=motion(ranges.tolist(),start,end)
    a=np.asarray(rows['rc'],float)
    report['rc_first_four']=[stats(a[:,i]) for i in range(1,a.shape[1])] if len(a) else []
    a=np.asarray(rows['timesync'],float)
    report['fc_boot_minus_host_monotonic_s']=stats(a[:,1]-a[:,0]) if len(a) else None
    output.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(output/'series.npz',**{k:np.asarray(v,float) for k,v in rows.items() if not k.startswith('/')},physical=np.asarray(physical))
    (output/'scans.json').write_text(json.dumps(scans),encoding='utf-8')
    (output/'uwb-events.json').write_text(json.dumps(selected_events),encoding='utf-8')
    report['events_sha256']=hashlib.sha256((capture/'events.jsonl').read_bytes()).hexdigest()
    (output/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path)
    parser.add_argument('--start',type=float,required=True,help='Host monotonic seconds')
    parser.add_argument('--end',type=float,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.end-args.start<10:parser.error('Need at least 10 seconds for separate endpoint medians')
    r=audit(args.capture,args.start,args.end,args.output)
    print(json.dumps({k:r[k] for k in ('uwb_xy','physical_B4','tof','attitude_deg','reasons') if k in r}))
