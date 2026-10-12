"""Bounded parallel matrix; each child creates its own network namespace."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
from queue import Queue
import subprocess
import sys


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runtime',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--instances',type=int,nargs=2,default=[52,53])
    p.add_argument('--seeds',type=int,nargs='+',default=[22,33,44,55,66])
    p.add_argument('--aiding-mode',choices=['both','enabled','disabled'],default='both')
    p.add_argument('--dwell-control',choices=['position','velocity'],default='position')
    p.add_argument('--lidar-source',choices=['truth_velocity','scan_matcher','spinning'],default='truth_velocity')
    p.add_argument('--no-deskew',action='store_true')
    p.add_argument('--no-spatial-support',action='store_true')
    p.add_argument('--sensor-capture-time',action='store_true')
    p.add_argument('--lidar-velocity-intervals',type=int,choices=[1,2],default=1)
    p.add_argument('--speed-factor',type=float,choices=[1.,2.],default=2.)
    p.add_argument('--hover-duration-s',type=float,choices=[2.,10.],default=2.)
    p.add_argument('--sitl-parameters',type=Path)
    p.add_argument('--fusion-horizon-ms',type=float,choices=[50.,100.,150.,200.],default=None)
    p.add_argument('--zero-unused-airspeed-delay',action='store_true')
    p.add_argument('--velocity-output-tau-s',type=float,choices=[.03,.05,.1,.25],default=None)
    p.add_argument('--scenarios',nargs='+',choices=['wall','scan_gap','unknown_front','clear','lidar_gap',
        'lidar_jump','lidar_stale','lidar_duplicate','lidar_bad_frame','lidar_bad_cov','lidar_reset',
        'uwb_gap','uwb_jump','person','partial_occlusion','imu_gap','clock_gap'],
        default=['clear','wall','scan_gap','unknown_front',
        'lidar_gap','lidar_jump','lidar_stale','lidar_duplicate','lidar_bad_frame',
        'lidar_bad_cov','lidar_reset','uwb_gap','uwb_jump'])
    args=p.parse_args()
    if len(set(args.instances))!=2 or any(not 50<=i<=59 for i in args.instances):
        p.error('two distinct private experiment instances in 50..59 are required')
    args.output.mkdir(parents=True,exist_ok=False)
    jobs=[]
    for scenario in args.scenarios:
        seeds=args.seeds if scenario in ('clear','wall') else args.seeds[:1]
        for seed in seeds:
            modes=((False,True) if scenario in ('clear','wall','uwb_gap','uwb_jump') else (True,))
            if args.aiding_mode!='both':modes=(args.aiding_mode=='enabled',)
            for aided in modes:
                jobs.append(dict(scenario=scenario,seed=seed,aided=aided))
    (args.output/'plan.json').write_text(json.dumps(dict(jobs=jobs,
        scope='synthetic UWB/flow/range and declared LiDAR source; propeller-to-wall >=0.50 m, scan loss brake <=0.30 s',
        lidar_source=args.lidar_source,lidar_velocity_intervals=args.lidar_velocity_intervals,
        sensor_capture_time=args.sensor_capture_time,deskew_enabled=not args.no_deskew,
        spatial_support_enabled=not args.no_spatial_support,
        hover_duration_s=args.hover_duration_s,speed_factor=args.speed_factor,
        held_out_synthetic_seeds=args.seeds,physical_writes=0),indent=2)+'\n')
    instances=Queue()
    for instance in args.instances:instances.put(instance)
    def run(item):
        instance=instances.get()
        name=f"{item['scenario']}-s{item['seed']}-a{int(item['aided'])}"
        command=[sys.executable,str(Path(__file__).with_name('obstacle_sitl.py')),
            '--runtime',str(args.runtime),'--output',str(args.output/name),'--patched-runtime',
            '--synthetic-flow','--scenario',item['scenario'],'--seed',str(item['seed']),
            '--duration','120','--speed-factor',str(args.speed_factor),'--instance',str(instance),
            '--lidar-source',args.lidar_source,'--lidar-velocity-intervals',str(args.lidar_velocity_intervals)]
        command.extend(['--dwell-control',args.dwell_control,'--hover-duration-s',str(args.hover_duration_s)])
        if args.sitl_parameters:command.extend(['--sitl-parameters',str(args.sitl_parameters)])
        if args.no_deskew:command.append('--no-deskew')
        if args.no_spatial_support:command.append('--no-spatial-support')
        if args.sensor_capture_time:command.append('--sensor-capture-time')
        if args.fusion_horizon_ms is not None:
            command.extend(['--fusion-horizon-ms',str(args.fusion_horizon_ms)])
        if args.zero_unused_airspeed_delay:command.append('--zero-unused-airspeed-delay')
        if args.velocity_output_tau_s is not None:
            command.extend(['--velocity-output-tau-s',str(args.velocity_output_tau_s)])
        if item['aided']:command.append('--lidar2d')
        # PX4 instance locks are in shared /tmp despite private networking.
        # Lease a distinct instance until its complete child process exits.
        try:
            with (args.output/(name+'.driver.log')).open('w') as stream:
                result=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT)
        finally:
            instances.put(instance)
        row=dict(item,run=name,returncode=result.returncode)
        print(json.dumps(row),flush=True)
        return row
    with ThreadPoolExecutor(max_workers=2) as pool:
        results=list(pool.map(run,jobs))
    (args.output/'results.json').write_text(json.dumps(results,indent=2)+'\n')
    if any(r['returncode'] for r in results):raise SystemExit(1)


if __name__=='__main__':main()
