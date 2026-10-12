"""Run three independent private-namespace SIH paths; preserve every raw run."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import subprocess
import sys
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    runner=Path(__file__).with_name('lidar2d_height_outage.py')
    scenarios=['nominal','range_gap','all_height_gap']
    seeds=[11,22,33,44,55]
    def path_runs(path,instance):
        results=[]
        for scenario in scenarios:
            for seed in seeds:
                name=f'{path}-{scenario}-seed{seed}'
                start=time.monotonic()
                cmd=[sys.executable,str(runner),'--runtime',str(args.runtime),'--output',str(args.output/name),
                     '--scenario',scenario,'--path',path,'--seed',str(seed),'--instance',str(instance)]
                with (args.output/(name+'.stdout')).open('w',encoding='utf-8') as out:
                    result=subprocess.run(cmd,stdout=out,stderr=subprocess.STDOUT)
                record=dict(name=name,returncode=result.returncode,wall_s=time.monotonic()-start)
                results.append(record)
                print(json.dumps(record),flush=True)
                if result.returncode:raise RuntimeError('failed run preserved: '+name)
        return results
    with ThreadPoolExecutor(max_workers=3) as pool:
        jobs=[pool.submit(path_runs,path,instance) for path,instance in
              [('lidar2d',71),('position_only',72),('legacy3d',73)]]
        results=[r for job in jobs for r in job.result()]
    (args.output/'matrix_status.json').write_text(json.dumps(results,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':main()
