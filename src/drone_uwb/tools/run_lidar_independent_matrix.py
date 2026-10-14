"""Run six complete, isolated SIH cases; all physical outputs are prohibited."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import subprocess
import sys


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--speed-factor',type=float,default=1.)
    parser.add_argument('--seeds',type=int,nargs='+',default=[11,22,33])
    parser.add_argument('--instance-base',type=int,default=82)
    args=parser.parse_args()
    if not 0 <= args.instance_base <= 99:
        parser.error('instance base must be between 0 and 99')
    args.output.mkdir(parents=True,exist_ok=False)
    runner=Path(__file__).with_name('lidar2d_independent_sih.py')
    def run_group(scenario,instance):
        for seed in args.seeds:
            name=f'{scenario}-seed{seed}'
            command=[sys.executable,str(runner),'--runtime',str(args.runtime),'--output',str(args.output/name),
                '--scenario',scenario,'--seed',str(seed),'--instance',str(instance),
                '--speed-factor',str(args.speed_factor)]
            with (args.output/(name+'.log')).open('x',encoding='utf-8') as log:
                subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
            print(name+' complete',flush=True)
    with ThreadPoolExecutor(max_workers=2) as executor:
        jobs=[executor.submit(run_group,scenario,instance) for scenario,instance in
              [('nominal',args.instance_base),('all_height_gap',args.instance_base+1)]]
        for job in jobs:job.result()


if __name__=='__main__':main()
