"""Render scientific review figures from completed saved analyses."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def render(root, destination):
    destination.mkdir(parents=True, exist_ok=True)
    rows=[json.loads(line) for line in (root/'replay_1845_imu_z_up_assumption/scans.jsonl').read_text(encoding='utf-8').splitlines()]
    t=np.array([r['stamp_ns'] for r in rows],dtype=np.int64)
    seconds=(t-t[0])/1e9
    chained=np.rad2deg([r['sequential_unwrapped_yaw_rad'] for r in rows])
    candidate=[];last_segment=None
    for row in rows:
        observation=row['candidate'];pose=observation['pose']
        candidate.append(float(np.rad2deg(pose[2])) if pose is not None and observation['segment']==last_segment else np.nan)
        last_segment=observation['segment']
    fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True,layout='constrained')
    axes[0].plot(seconds,chained,color='#b64f40',label='New sequential comparator')
    axes[1].plot(seconds,candidate,color='#166b86',label='Keyframe + IMU (segment-relative)')
    for axis in axes:
        axis.set_ylabel('Yaw (degrees)');axis.grid(alpha=.2);axis.legend(loc='lower left')
    axes[1].set_ylim(-1,1);axes[1].set_xlabel('Recorded scan time since first sample (s)')
    fig.suptitle('18:45 RAW replay: yaw stability diagnostic\nDifferent vertical scales; no independent rotation truth',fontsize=13)
    fig.savefig(destination/'raw_yaw_diagnostic.png',dpi=180);plt.close(fig)

    results=json.loads((root/'sitl_50cm_20261012.json').read_text(encoding='utf-8'))['results']
    wall=[r for r in results if r['scenario']=='wall'];seeds=sorted({r['seed'] for r in wall})
    fig,ax=plt.subplots(figsize=(10,5.5),layout='constrained')
    x=np.arange(len(seeds));width=.34
    for aided,label,color,offset in ((False,'UWB + flow + range','#89959c',-width/2),(True,'Same + synthetic LiDAR XY velocity','#187a8d',width/2)):
        values=[next(r['status']['min_wall_surface_clearance_m'] for r in wall if r['seed']==seed and r['lidar_aiding']==aided) for seed in seeds]
        bars=ax.bar(x+offset,values,width,label=label,color=color)
        ax.bar_label(bars,labels=[f'{v:.3f} m' for v in values],padding=4,fontsize=10)
    ax.axhline(.5,color='#b64f40',linestyle='--',label='Required outer-envelope gap: 0.50 m')
    ax.set_xticks(x,[f'Seed {seed}' for seed in seeds]);ax.set_ylim(0,1.1)
    ax.set_ylabel('Minimum outer-envelope to wall gap (m)')
    ax.set_title('Isolated SIH wall approach: 0.20 m assumed vehicle radius',fontsize=13)
    ax.legend(loc='upper right',fontsize=9);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
    fig.supxlabel('Synthetic observations, 3 repeats per condition. Clear-route completion: baseline 1/3, aided 0/3.\nThese wall results do not establish overall mission readiness or real-flight safety.',fontsize=9)
    fig.savefig(destination/'sitl_wall_clearance_50cm.png',dpi=180);plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--outputs',type=Path,required=True)
    p.add_argument('--destination',type=Path,required=True)
    a=p.parse_args();render(a.outputs,a.destination)
