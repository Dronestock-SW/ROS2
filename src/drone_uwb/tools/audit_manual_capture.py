"""Read a hash-verified capture; distinguish observed flight from EKF fusion."""
import argparse
from bisect import bisect_right
from collections import Counter, defaultdict
from datetime import datetime, timezone, timedelta
import hashlib
import json
import math
from pathlib import Path
import struct

import numpy as np


def kst(ns):
    return datetime.fromtimestamp(ns/1e9, timezone.utc).astimezone(
        timezone(timedelta(hours=9))).isoformat()


def quantiles(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    return None if not len(a) else dict(n=len(a), minimum=float(a.min()),
        median=float(np.median(a)), p95=float(np.quantile(a, .95)), maximum=float(a.max()))


def rate(rows, field):
    stamps = [r[field] for r in rows if isinstance(r.get(field), int)]
    gaps = np.diff(stamps)/1e9
    return dict(count=len(stamps), unique_count=len(set(stamps)),
        nonincreasing=int(np.sum(gaps <= 0)),
        span_s=(stamps[-1]-stamps[0])/1e9 if len(stamps)>1 else None,
        hz=(len(stamps)-1)*1e9/(stamps[-1]-stamps[0]) if len(stamps)>1 and stamps[-1]>stamps[0] else None,
        gaps_s=quantiles(gaps))


def content(row):
    value = row['data']
    if row['type'] == 'std_msgs/msg/String':
        try:return json.loads(value['data'])
        except (KeyError, ValueError):return dict(parse_error=True)
    return value


def decode_parameter(row):
    """PARAM_VALUE wire layout; PX4 INT32 is bytewise, not float-cast.

    The recorder's FRAMING_OK is required. This is a recorded parameter read,
    not evidence that any observation was received or fused by the EKF.
    """
    d = row['data']
    if (d.get('msgid'), d.get('sysid'), d.get('compid'), d.get('framing_status')) != (22,1,1,1):
        return None
    payload = b''.join(struct.pack('<Q', x) for x in d['payload64'])[:d['len']]
    if len(payload) < 25:return None
    name = payload[8:24].split(b'\0')[0].decode('ascii')
    if payload[24] == 6:value = struct.unpack('<i', payload[:4])[0]
    elif payload[24] == 9:value = struct.unpack('<f', payload[:4])[0]
    else:return None
    return dict(name=name, value=value, type=payload[24], wire_hex=payload[:4].hex(),
                line=row['line'], received_kst=kst(row['received_ros_ns']))


def audit(path, expected_sha256):
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != expected_sha256:
        raise ValueError('source_sha256_mismatch: refusing incomplete or changed raw data')
    topics = defaultdict(list)
    parameters=[]
    message_ids=Counter()
    with path.open(encoding='utf-8') as stream:
        for line, text in enumerate(stream, 1):
            row=json.loads(text);row['line']=line
            if row['topic']=='/uas1/mavlink_source':
                message_ids[row['data']['msgid']]+=1
                parameter=decode_parameter(row)
                if parameter:parameters.append(parameter)
            else:topics[row['topic']].append(row)
    all_rows=sorted((r for rows in topics.values() for r in rows),
                    key=lambda r:r['received_monotonic_ns'])
    latest={};selected=[]
    for row in all_rows:
        now=row['received_monotonic_ns'];latest[row['topic']]=row
        if row['topic']!='/mavros/local_position/odom':continue
        state,land=latest.get('/mavros/state'),latest.get('/mavros/extended_state')
        if not state or not land:continue
        d,e=content(state),content(land)
        if (0 <= now-state['received_monotonic_ns'] <= 1_500_000_000
                and 0 <= now-land['received_monotonic_ns'] <= 500_000_000
                and d.get('connected') and d.get('armed') and d.get('mode')=='POSCTL'
                and e.get('landed_state')==2):selected.append(row)
    groups=[]
    for row in selected:
        if not groups or (row['received_monotonic_ns']-groups[-1][-1]['received_monotonic_ns'])/1e9 > .15:
            groups.append([])
        groups[-1].append(row)
    segments=[]
    for rows in groups:
        positions=np.array([[r['data']['pose']['pose']['position'][k] for k in ('x','y')] for r in rows])
        segments.append(dict(first_observed_kst=kst(rows[0]['received_ros_ns']),
            last_observed_kst=kst(rows[-1]['received_ros_ns']),first_line=rows[0]['line'],last_line=rows[-1]['line'],
            first_monotonic_ns=rows[0]['received_monotonic_ns'],last_monotonic_ns=rows[-1]['received_monotonic_ns'],
            odom_samples=len(rows),observed_span_s=(rows[-1]['received_monotonic_ns']-rows[0]['received_monotonic_ns'])/1e9,
            fc_xy_rms_about_own_mean_m=float(np.sqrt(np.mean(np.sum((positions-positions.mean(axis=0))**2,axis=1)))),
            fc_xy_start_to_end_m=float(np.linalg.norm(positions[-1]-positions[0])),
            scope='observed simultaneous fresh ARM/POSCTL/landed_state=IN_AIR; not necessarily stick-neutral hover or physical accuracy'))
    scan_ratios=[];scan_ages=[]
    for r in topics['/scan']:
        d=r['data'];period=d.get('scan_time',0);dt=d.get('time_increment',0)
        if isinstance(period,(int,float)) and period>0 and isinstance(dt,(int,float)):
            scan_ratios.append((len(d['ranges'])-1)*dt/period)
        if r.get('header_ns'):scan_ages.append((r['received_ros_ns']-r['header_ns'])/1e9)
    offsets=[(r['received_ros_ns']-r['received_monotonic_ns'])/1e9 for r in all_rows]
    bridge=[dict(line=r['line'],received_kst=kst(r['received_ros_ns']),status=content(r))
            for topic in ('/uwb/local_odometry_status','/uwb/bridge_status') for r in topics.get(topic,[])]
    btf=topics.get('/uwb/btf_status',[])
    sync=topics.get('/mavros/timesync_status',[])
    return dict(schema=1,source=str(path.resolve()),sha256=digest,bytes=path.stat().st_size,
        scope='recorded ROS and MAVLink receive stream; not independent ground truth',
        first_received_kst=kst(all_rows[0]['received_ros_ns']),last_received_kst=kst(all_rows[-1]['received_ros_ns']),
        receipt_clock_offset_span_s=max(offsets)-min(offsets),
        topic_rates={t:dict(receipt=rate(r,'received_monotonic_ns'),measurement_header=rate(r,'header_ns')) for t,r in topics.items()},
        mavlink_message_counts=dict(message_ids),parameters=parameters,manual_position_in_air_segments=segments,
        scan=dict(time_span_over_period=quantiles(scan_ratios),receive_minus_header_s=quantiles(scan_ages),
                  note='Acquisition duration is included; this is not EKF delay.'),
        timesync=dict(samples=len(sync),rtt_ms=quantiles([r['data']['round_trip_time_ms'] for r in sync]),
            estimated_offset_span_ns=quantiles([r['data']['estimated_offset_ns'] for r in sync]),
            observed_minus_estimated_ms=quantiles([(r['data']['observed_offset_ns']-r['data']['estimated_offset_ns'])/1e6 for r in sync])),
        btf_fc_output_enabled_counts=dict(Counter(str(content(r).get('fc_output_enabled')) for r in btf)),
        bridge_status_records=bridge,
        sender_jump_gate_off_proven=False,actual_ev_fusion_proven=False,
        fusion_limitation='PARAM_VALUE and ESTIMATOR_STATUS validity do not prove actual EV sample fusion. Match ULog aid-source samples.',
        physical_commands_sent=0)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--events',type=Path,required=True);p.add_argument('--sha256',required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();result=audit(args.events,args.sha256)
    result['script_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(dict(output=str(args.output),segments=result['manual_position_in_air_segments'],
                         parameters=result['parameters']),ensure_ascii=False))


if __name__=='__main__':main()
