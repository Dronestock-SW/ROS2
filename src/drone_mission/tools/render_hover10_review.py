"""Show fixed-window hover errors and the unchanged velocity criteria."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def data(root):
    rows=[json.loads(s) for s in (root/'mission.jsonl').read_text(encoding='utf-8').splitlines()]
    rows=[r for r in rows if r.get('phase_before_tick')=='DWELL']
    t=np.array([r['clock_s'] for r in rows]);t-=t[0]
    values=np.array([[np.linalg.norm(np.array(r['pose_xy'])-r['target_before_tick']),
        np.linalg.norm(np.array(r['truth_xy'])-r['target_before_tick']),
        np.linalg.norm(r['velocity_xy']),abs(r['snapshot']['vertical_speed_m_s'])] for r in rows])
    return t,values


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--before',type=Path,required=True)
    p.add_argument('--after',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();fig,axes=plt.subplots(3,2,figsize=(12,8.5),sharey='row',sharex='col')
    for col,(root,title) in enumerate(((a.before,'Previous candidate / seed 201'),
                                      (a.after,'Scan-derived velocity / held-out seed 301'))):
        t,y=data(root)
        for row,(index,label,limit) in enumerate(((0,'Distance to fixed XY goal [m]',.15),
                                                (2,'FC horizontal speed [m/s]',.1),
                                                (3,'Absolute FC vertical speed [m/s]',.1))):
            ax=axes[row,col];ax.plot(t,y[:,index],lw=1,color='#2368a0',label='FC estimate')
            ax.axhline(limit,color='#b83e3e',ls='--',lw=1.1,label='Continuous criterion')
            ax.axvspan(0,10,color='#d8e7f2',alpha=.45,label='Preselected first 10 s')
            if row==0:
                ax.plot(t,y[:,1],lw=.9,color='#3b9479',alpha=.8,label='SIH truth (telemetry)')
                ax.set_title(title,fontsize=11);ax.legend(fontsize=7.5,loc='upper right')
            ax.set_ylabel(label);ax.set_ylim(bottom=0);ax.grid(alpha=.2)
        axes[-1,col].set_xlabel('Seconds since arrival')
    fig.suptitle('Better XY holding does not establish a continuous 10-second pass',fontsize=14)
    fig.text(.5,.017,'Both examples failed the unchanged velocity criteria. Different noise seeds; simulation, not physical flight.',
             ha='center',fontsize=9)
    fig.tight_layout(rect=(0,.04,1,.96));fig.savefig(a.output,dpi=170);plt.close(fig)
