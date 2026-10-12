"""Render before/after hover criteria without conflating position and completion."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def series(root):
    rows=[json.loads(s) for s in (root/'mission.jsonl').read_text(encoding='utf-8').splitlines()]
    poses={}
    for line in (root/'telemetry.jsonl').read_text(encoding='utf-8').splitlines():
        r=json.loads(line);p=r['message']
        if p['mavpackettype']=='LOCAL_POSITION_NED':poses[p['time_boot_ms']]=p
    rows=[r for r in rows if r.get('phase_before_tick',r['phase'])=='DWELL']
    time=np.array([r['clock_s'] for r in rows]);values=[]
    for r in rows:
        p=poses[round(r['clock_s']*1000)]
        target=r.get('target_before_tick',r.get('mission_target_xy',[2.9,2.]))
        values.append([np.linalg.norm(np.array(r['pose_xy'])-target),
            np.linalg.norm(np.array(r['truth_xy'])-target),
            np.linalg.norm(r['velocity_xy']),abs(p['vz'])])
    return time-time[0],np.array(values)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--before',type=Path,required=True);p.add_argument('--after',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    fig,axes=plt.subplots(3,2,figsize=(12,8.5),sharex='col',sharey='row')
    for col,(root,title) in enumerate(((args.before,'Previous failure / seed 101'),
                                      (args.after,'Improved candidate: remaining failure / seed 201'))):
        t,y=series(root)
        for row,(index,label,limit) in enumerate(((0,'XY distance to goal [m]',.15),
                                                (2,'FC horizontal speed [m/s]',.1),
                                                (3,'Absolute FC vertical speed [m/s]',.1))):
            ax=axes[row,col];ax.plot(t,y[:,index],color='#2468a0',lw=1.3,label='FC estimate')
            ax.axhline(limit,color='#b12b35',ls='--',lw=1.2,label='Unchanged criterion')
            if row==0:ax.plot(t,y[:,1],color='#46a688',lw=1.,alpha=.8,label='Simulator truth')
            ax.set_ylabel(label);ax.grid(alpha=.2);ax.set_ylim(bottom=0)
            if row==0:ax.set_title(title,fontsize=11);ax.legend(fontsize=8)
        axes[2,col].set_xlabel('Time since hover evaluation began [s]')
    fig.suptitle('XY holding improved; every criterion must still remain valid for 2 seconds',fontsize=14)
    fig.text(.5,.017,'Diagnostic examples, different synthetic noise seeds; not a paired accuracy comparison. Both runs timed out.',
             ha='center',fontsize=9,color='#444444')
    fig.tight_layout(rect=(0,.04,1,.96));fig.savefig(args.output,dpi=170);plt.close(fig)
