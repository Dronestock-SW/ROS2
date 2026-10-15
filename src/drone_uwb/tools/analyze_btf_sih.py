"""Separate MAVLink send, FC receipt evidence and actual EKF fusion evidence."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from pyulog import ULog


def analyze(run):
    paths=list(run.rglob('*.ulg'))
    if len(paths)!=1:raise ValueError('exactly one ULog required')
    u=ULog(str(paths[0]))
    if u.file_corruption or u.dropouts:raise ValueError('invalid ULog evidence')
    sent=[json.loads(line) for line in (run/'sent.jsonl').open(encoding='utf-8')]
    xy=np.array([[r['x'],r['y']] for r in sent])
    stamps=np.array([r['capture_us'] for r in sent],dtype=np.int64)
    visual=u.get_dataset('vehicle_visual_odometry').data
    aid=u.get_dataset('estimator_aid_src_ev_pos').data
    flags=u.get_dataset('estimator_status_flags').data
    status=u.get_dataset('vehicle_status').data
    deltas=[]
    for i in range(len(visual['timestamp'])):
        observed=np.array([visual['position[0]'][i],visual['position[1]'][i]])
        error=np.linalg.norm(xy-observed,axis=1);j=int(np.argmin(error))
        if error[j]<=2e-6:deltas.append(int(visual['timestamp_sample'][i])-int(stamps[j]))
    fuse_times=np.unique(aid['time_last_fuse'][aid['fused'].astype(bool)].astype(np.int64))
    def distribution(values):
        return dict(zip(('min','p50','p95','max'),map(float,np.percentile(values,[0,50,95,100])))) if len(values) else None
    summary=dict(scope='isolated disarmed SIH, recorded UWB XY and arrival gaps; not a flight or live-FC result',
        ulog_sha256=hashlib.sha256(paths[0].read_bytes()).hexdigest(),
        corrupted=False,dropouts=0,sent=len(sent),
        visual_odometry_logged=len(visual['timestamp']),visual_xy_matched_to_wire=len(deltas),
        fc_sample_minus_source_capture_us=distribution(deltas),
        receipt_to_sample_age_us=distribution(visual['timestamp'].astype(np.int64)-visual['timestamp_sample'].astype(np.int64)),
        ekf_ev_position_reports=len(aid['timestamp']),ekf_fused_reports=int(np.count_nonzero(aid['fused'])),
        ekf_rejected_reports=int(np.count_nonzero(aid['innovation_rejected'])),
        distinct_fusion_times=len(fuse_times),max_complete_fusion_gap_s=float(np.max(np.diff(fuse_times))/1e6) if len(fuse_times)>1 else None,
        ev_position_active=bool(np.any(flags['cs_ev_pos'])),
        armed=bool(np.any(status['arming_state']==2)),
        timesync_responses=json.loads((run/'result.json').read_text()).get('timesync_responses',0))
    if not summary['ev_position_active'] or summary['ekf_fused_reports']==0 or summary['armed']:
        raise ValueError('disarmed EV fusion proof failed')
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();summary=analyze(a.run)
    a.output.write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(summary))
