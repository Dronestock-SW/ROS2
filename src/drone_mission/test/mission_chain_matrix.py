"""Run each actual PX4 virtual mission in sequence and retain every outcome."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

SCENARIOS = ('nominal','spike','nlos','coherent_step','phase_delay',
             'short_gap','long_gap','tof_short_gap','tof_long_gap','scan_missing','manual',
             'scan_partial','scanner_missing_partial','scan_failed_partial')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--scenarios',nargs='+',choices=SCENARIOS,default=SCENARIOS)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    results=[]
    runner=Path(__file__).with_name('mission_chain_px4.py')
    for scenario in args.scenarios:
        print('START',scenario,flush=True)
        with (args.output/(scenario+'-driver.log')).open('w') as log:
            code=subprocess.call([sys.executable,str(runner),'--px4-root',str(args.px4_root),
                                  '--scenario',scenario,'--output',str(args.output/scenario)],
                                 stdout=log,stderr=subprocess.STDOUT)
        summary=args.output/scenario/'summary.json'
        result=dict(scenario=scenario,exit_code=code,
                    summary=json.loads(summary.read_text()) if summary.exists() else None)
        results.append(result)
        (args.output/'matrix.json').write_text(json.dumps(results,indent=2)+'\n',encoding='utf-8')
        print('PASS' if code==0 else 'FAIL',scenario,flush=True)
        # A failed nominal trial cannot justify proceeding to fault trials.
        if scenario=='nominal' and code:
            break
    return 0 if len(results)==len(args.scenarios) and all(r['exit_code']==0 for r in results) else 1


if __name__=='__main__':
    sys.exit(main())
