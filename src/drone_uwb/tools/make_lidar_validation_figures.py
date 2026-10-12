"""Plot recorded offline results; does not recompute or substitute test data."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--evidence',type=Path,required=True)
args=p.parse_args()
evidence=args.evidence
synthetic=json.loads((evidence/'synthetic_final.json').read_text(encoding='utf-8'))
lines=(evidence/'reset_probe.log').read_text(encoding='utf-8-sig').splitlines()
resets=[json.loads(x.removeprefix('RESULT ')) for x in lines if x.startswith('RESULT ')]
lever=[json.loads(x.removeprefix('LEVER ')) for x in lines if x.startswith('LEVER ')]
vertical=[json.loads(x.removeprefix('VERTICAL_DISPATCH ')) for x in lines if x.startswith('VERTICAL_DISPATCH ')]
(evidence/'reset_probe.json').write_text(json.dumps(dict(reset=resets,lever_arm=lever,vertical_dispatch=vertical),indent=2)+'\n',encoding='utf-8')

cases=['static','xy','yaw','mixed','corridor','occlusion','angular_perturbation']
labels=['Static','X/Y motion','Yaw motion','Mixed','Parallel walls','Occlusion','Angle fault']
counts=[sum(r['accepted'] for r in synthetic['results'] if r['case']==c) for c in cases]
totals=[sum(r['intervals'] for r in synthetic['results'] if r['case']==c) for c in cases]
fig,axes=plt.subplots(1,2,figsize=(12.8,5.3),layout='constrained')
fig.suptitle('Horizontal LiDAR candidate: observed scope and reset hazard',fontsize=15,weight='bold')
colors=['#177f78']*4+['#a1433c','#aa7833','#aa7833']
axes[0].barh(labels,np.array(counts)/totals*100,color=colors,height=.65)
axes[0].invert_yaxis()
axes[0].set_xlim(0,125)
axes[0].set_xticks([0,25,50,75,100])
axes[0].set_xlabel('Accepted intervals (%)')
axes[0].set_title('70 synthetic runs: 10 seeds per scene',fontsize=12)
for i,(n,total) in enumerate(zip(counts,totals)):
    axes[0].text(102,i,f'{n:,}/{total:,}',va='center',fontsize=9)
axes[0].grid(axis='x',alpha=.18)
rows=sorted([r for r in resets if r['axis']=='pitch' and r['supplied_vz_variance']==10000],key=lambda r:r['angle_deg'])
angles=[r['angle_deg'] for r in rows]
for i,label,color in [(0,'Horizontal X variance','#177f78'),(2,'Vertical Z variance','#a1433c')]:
    axes[1].plot(angles,[r['after_variance'][i] for r in rows],'-o',label=label,color=color,lw=2)
axes[1].set_yscale('log')
axes[1].set_xticks(angles)
axes[1].set_xlabel('Aligned pitch (degrees)')
axes[1].set_ylabel('Velocity variance after reset ((m/s)^2, log scale)')
axes[1].set_title('Actual PX4 EKF call; supplied Z variance = 10,000',fontsize=12)
axes[1].legend(loc='lower right',fontsize=9)
axes[1].grid(alpha=.18)
fig.supxlabel('Offline synthetic scans / direct EKF calls only. Original capture replay and flight validation remain incomplete.',fontsize=9)
fig.savefig(evidence/'offline_results.png',dpi=170)
plt.close(fig)

trace_file=evidence/'vertical_trace_seed11.json'
if trace_file.exists():
    traces=json.loads(trace_file.read_text(encoding='utf-8'))
    fig,axes=plt.subplots(2,2,figsize=(12.8,7.5),layout='constrained',sharex=True)
    fig.suptitle('Same-commit SIH: loss of height aiding while stationary and DISARM',fontsize=15,weight='bold')
    for name,label,color in [('vertical_aiding','Position + body velocity (vz = 0, variance = 10,000)','#ae3f38'),
                             ('vertical_aiding-position-only','Position only','#177f78')]:
        d=traces['runs'][name]['trace']
        t=d['time_s']
        for ax,field in zip(axes.flat,['z','vz','vz_resets_cumulative','z_valid']):
            ax.plot(t,d[field],label=label,color=color,lw=1.4)
    d=traces['runs']['vertical_aiding']['trace']
    axes[0,0].plot(d['time_s'],d['truth_z'],'--',color='#27343c',lw=1,label='Simulator truth')
    titles=['Local Z output (NED, m)','Local Vz output (m/s)','Cumulative vertical velocity resets','Z valid flag']
    for ax,title in zip(axes.flat,titles):
        ax.set_title(title,fontsize=11)
        ax.axvline(40.008,color='#66747d',ls=':',lw=1.4)
        ax.grid(alpha=.18)
        ax.set_xlim(0,120)
    axes[1,1].set_yticks([0,1])
    axes[1,1].set_ylim(-.1,1.1)
    axes[0,0].legend(fontsize=8,loc='lower left')
    for ax in axes[1]:
        ax.set_xlabel('Simulation elapsed time (s)')
    fig.supxlabel('Seed 11; baro/range controls disabled at 40.008 s. Plotted at ~10 Hz; metrics use full ULog. No airborne failsafe test.',fontsize=9)
    fig.savefig(evidence/'vertical_aiding_seed11.png',dpi=170)
    plt.close(fig)
