"""Export one paired stationary vertical-aiding trace from unchanged ULogs."""
import argparse
import json
from pathlib import Path

import numpy as np
from pyulog import ULog

from lidar_ev_sitl import sha256


def trace(run):
    logfile = next(run.rglob('*.ulg'))
    u = ULog(str(logfile))
    data = {d.name: d.data for d in u.data_list if d.multi_id == 0}
    base = json.loads((run/'run_status.json').read_text(encoding='utf-8'))['first_clock_us']
    local = data['vehicle_local_position']
    truth = data['vehicle_local_position_groundtruth']
    increments = np.r_[0, np.diff(local['vz_reset_counter'].astype(np.int64)) % 256]
    cumulative = np.cumsum(increments)
    # Keep the last publication in each 100 ms bin, retaining original times.
    bins = ((local['timestamp'].astype(float)-base)/100000).astype(int)
    selected = np.r_[np.diff(bins) != 0, True] & (local['timestamp'] >= base)
    timestamps = local['timestamp'][selected]
    fields = {k:local[k][selected].astype(float).tolist()
              for k in ['z','vz','z_valid','v_z_valid','vz_reset_counter']}
    fields['vz_resets_cumulative'] = cumulative[selected].tolist()
    for key in ['z','vz']:
        fields['truth_'+key] = np.interp(timestamps,truth['timestamp'],truth[key]).tolist()
    fields['time_s'] = ((timestamps.astype(float)-base)/1e6).tolist()
    changes = [dict(time_s=(t-base)/1e6,name=name,value=value)
               for t,name,value in u.changed_parameters if name in ['EKF2_BARO_CTRL','EKF2_RNG_CTRL']]
    return dict(ulog_sha256=sha256(logfile),firmware=u.msg_info_dict['ver_sw'],
                changed_height_parameters=changes,trace=fields,
                scope='stationary DISARM; decimated local-position output, not flight')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--matrix',type=Path,required=True)
    p.add_argument('--seed',type=int,default=11)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    result={name:trace(a.matrix/f'{name}-seed{a.seed}')
            for name in ['vertical_aiding','vertical_aiding-position-only']}
    a.output.write_text(json.dumps(dict(seed=a.seed,runs=result),indent=2,allow_nan=False)+'\n',encoding='utf-8')
    for name,r in result.items():
        d=r['trace']
        print(name,'last_z',d['z'][-1],'last_vz',d['vz'][-1],
              'last_truth_z',d['truth_z'][-1],'vz_resets',d['vz_resets_cumulative'][-1],
              'parameter_changes',r['changed_height_parameters'])
