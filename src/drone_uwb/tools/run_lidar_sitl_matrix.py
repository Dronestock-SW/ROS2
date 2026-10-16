"""Run stationary same-commit transport diagnostics, not the full S01-S14 mission."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
from pathlib import Path
import subprocess
import sys

from analyze_lidar_sitl import analyze


def cases():
    result = []
    for mode in [0,1]:
        for variance in [1,100,10000,1000000]:
            result.append((f'variance-m{mode}-z{variance}', ['--noise-mode',str(mode),'--z-variance',str(variance)]))
    for scenario in ['lidar_gap','uwb_gap']:
        for gap in [.2,1,5]:
            result.append((f'{scenario}-{gap}', ['--scenario',scenario,'--gap',str(gap)]))
    for scenario in ['nan_vz','invalid_velocity_frame','reset','quality','duplicate','reverse_time','clock_jump','alternating','stale_velocity','vertical_aiding']:
        result.append((scenario,['--scenario',scenario]))
    for delay in [50,100,150]:
        result.append((f'delay-{delay}',['--scenario','delay','--delay-ms',str(delay)]))
    result.append(('vertical_aiding-position-only',['--scenario','vertical_aiding','--position-only']))
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--px4-root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--workers', type=int, default=4)
    p.add_argument('--speed-factor', type=float, default=4)
    p.add_argument('--seeds', type=int, nargs='+', default=[11,22,33,44,55])
    args = p.parse_args()
    if not 1 <= args.workers <= 5:
        p.error('workers must be 1..5')
    args.output.mkdir(parents=True, exist_ok=False)
    scripts = Path(__file__).resolve().parent
    jobs = [(name, seed, extra) for name,extra in cases() for seed in args.seeds]
    queues = [jobs[i::args.workers] for i in range(args.workers)]
    (args.output/'matrix_plan.json').write_text(json.dumps(dict(jobs=jobs, speed_factor=args.speed_factor,
         duration_s=120, workers=args.workers, scope=__doc__), indent=2)+'\n', encoding='utf-8')

    def worker(slot):
        results=[]
        for name,seed,extra in queues[slot]:
            target = args.output/f'{name}-seed{seed}'
            command = [sys.executable,str(scripts/'lidar_ev_sitl.py'), '--px4-root',str(args.px4_root),
                       '--output',str(target),'--duration','120','--seed',str(seed),
                       '--speed-factor',str(args.speed_factor),'--instance',str(20+slot),*extra]
            with (args.output/f'{target.name}.driver.log').open('w',encoding='utf-8') as stream:
                finished = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT)
            record=dict(name=name,seed=seed,returncode=finished.returncode,path=str(target))
            if finished.returncode == 0:
                try:
                    analysis=analyze(target)
                    record['summary']=dict(sent=analysis['status']['odometry_sent'],
                                           phase=analysis['phase'],timesync=analysis['timesync'])
                except Exception as exc:
                    record['analysis_error']=repr(exc)
            results.append(record)
            print(json.dumps({k:v for k,v in record.items() if k!='summary'}), flush=True)
        return results

    results=[]
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        for completed in as_completed([pool.submit(worker,i) for i in range(args.workers)]):
            results.extend(completed.result())
    (args.output/'matrix_results.json').write_text(json.dumps(dict(results=results), indent=2,allow_nan=False)+'\n',encoding='utf-8')


if __name__ == '__main__':
    main()
