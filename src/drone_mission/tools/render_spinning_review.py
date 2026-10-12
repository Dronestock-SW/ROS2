"""Static comparison of fixed input scenarios and all held-out clear seeds."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--summary',type=Path,required=True)
    p.add_argument('--output-directory',type=Path,required=True)
    a=p.parse_args();data=json.loads(a.summary.read_text(encoding='utf-8'))
    a.output_directory.mkdir(parents=True,exist_ok=True)
    scenarios=['clear','constant_twist','right_person','partial_right','imu_gap']
    labels=['Varying rotation','Constant twist','Moving person','Partial sector','IMU gap']
    modes=[('full','Deskew + support','#2368a0'),('no_deskew','No deskew','#cc7448'),
        ('no_support','No spatial/temporal checks','#63987a')]
    fig,axes=plt.subplots(1,2,figsize=(12.8,4.6))
    for ax,key,scale,title in [(axes[0],'velocity_rms_m_s',100,'Interval velocity RMS [cm/s]'),
                               (axes[1],'withholding_fraction',100,'Withheld scans [%]')]:
        for i,(mode,label,color) in enumerate(modes):
            for j,scenario in enumerate(scenarios):
                rows=[r for r in data['frontend'] if r['mode']==mode and r['scenario']==scenario]
                y=[r[key]*scale for r in rows];x=j+(i-1)*.23
                ax.bar(x,np.mean(y),width=.21,color=color,label=label if j==0 else None,alpha=.85)
                ax.scatter([x]*len(y),y,s=16,color='#243441',zorder=4)
        ax.set_xticks(np.arange(len(labels)),labels,rotation=20,ha='right')
        ax.set_ylabel(title);ax.set_ylim(bottom=0);ax.grid(axis='y',alpha=.2)
    axes[0].legend(fontsize=8)
    fig.suptitle('Same synthetic rays and IMU per seed: 54 cases, 116 scans per case',fontsize=13)
    fig.text(.5,.01,'Points: seeds 901 / 902 / 903. RMS uses accepted samples; withholding includes startup. '
        'Parallel-wall cases: 0 / 116 published in every mode.',ha='center',fontsize=8)
    fig.tight_layout(rect=(0,.05,1,.94))
    fig.savefig(a.output_directory/'spinning_frontend.png',dpi=170);plt.close(fig)
    clear=[r for r in data['runs'] if r['scenario']=='clear']
    modes=[(True,True,'Deskew + aiding','#2368a0'),(False,True,'Aiding disabled','#63987a'),
        (True,False,'Deskew disabled','#cc7448')]
    fig,axes=plt.subplots(1,2,figsize=(11.6,4.4),sharey=True)
    for ax,key,title in [(axes[0],'fc_goal_xy_m','FC estimate'),(axes[1],'truth_goal_xy_m','Native SIH truth')]:
        for i,(aided,deskew,label,color) in enumerate(modes):
            rows=sorted([r for r in clear if r['aided']==aided and r['deskew']==deskew],key=lambda r:r['seed'])
            ax.bar(np.arange(len(rows))+(i-1)*.24,[r['first_10s_ulog'][0][key]['rms']*100 for r in rows],
                width=.22,color=color,label=label)
        ax.axhline(3.,ls='--',color='#b83e3e',label='3 cm requirement')
        ax.set_xticks(range(3),['501','502','503']);ax.set_xlabel('Held-out seed');ax.set_title(title)
        ax.grid(axis='y',alpha=.2)
    axes[0].set_ylabel('XY RMS from fixed goal [cm]')
    handles,legend_labels=axes[1].get_legend_handles_labels()
    fig.legend(handles,legend_labels,fontsize=8,loc='upper center',bbox_to_anchor=(.5,.90),ncol=4)
    fig.suptitle('Predeclared first 10 seconds after arrival; no mean subtraction',fontsize=13)
    fig.text(.5,.01,'Same profile and fixture; separate closed-loop runs, not identical physics replay. '
        'All clear runs failed continuous 10-second stability.',ha='center',fontsize=8)
    fig.tight_layout(rect=(0,.045,1,.81))
    fig.savefig(a.output_directory/'spinning_hover.png',dpi=170);plt.close(fig)
    run=next(r for r in clear if r['seed']==501 and r['aided'] and r['deskew'])
    curve=run['stability']['curve'];t=[r['time_s'] for r in curve]
    fig,axes=plt.subplots(3,1,figsize=(10,7),sharex=True)
    for ax,key,limit,title in zip(axes,['fc_xy_m','speed_m_s','vz_m_s'],[.15,.1,.1],
        ['XY goal distance [m]','FC horizontal speed [m/s]','FC |vz| [m/s]']):
        ax.plot(t,[r[key] for r in curve],color='#2368a0',lw=1,label='FC')
        ax.axhline(limit,color='#b83e3e',ls='--',label='Continuous criterion')
        ax.axvspan(0,10,color='#d8e7f2',alpha=.5);ax.set_ylabel(title);ax.grid(alpha=.2)
        ax.set_ylim(bottom=0)
    axes[0].plot(t,[r['truth_xy_m'] for r in curve],color='#63987a',lw=.8,label='Truth telemetry')
    axes[0].legend(fontsize=8);axes[-1].set_xlabel('Seconds since arrival')
    fig.suptitle('Held-out seed 501: speed peaks interrupt the stability timer',fontsize=13)
    fig.tight_layout(rect=(0,0,1,.94))
    fig.savefig(a.output_directory/'spinning_stability.png',dpi=170);plt.close(fig)


if __name__=='__main__':main()
