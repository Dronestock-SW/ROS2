"""Hash immutable hover trials and preserve development/validation denominators."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def group(root,stage):
    plan=json.loads((root/'plan.json').read_text(encoding='utf-8'))
    results=json.loads((root/'results.json').read_text(encoding='utf-8'))
    if len(plan['jobs'])!=len(results):raise ValueError('incomplete trial group')
    rows=[];bootstrap_failures=[]
    for result in results:
        run=root/result['run']
        if not run.exists() and result['returncode']!=0:
            driver=root/(result['run']+'.driver.log')
            bootstrap_failures.append(dict(**result,before_output_creation=True,
                driver_log_sha256=sha(driver)))
            continue
        manifest=json.loads((run/'manifest.json').read_text(encoding='utf-8'))
        status=json.loads((run/'run_status.json').read_text(encoding='utf-8'))
        files=[run/name for name in ('manifest.json','run_status.json','mission.jsonl','telemetry.jsonl')]
        files += [run/name for name in status['ulog_files']]
        files += [p for p in (run/'scan_observations.jsonl',) if p.exists()]
        files += sorted((run/'source_snapshot').glob('*'))
        rows.append(dict(**result,status=status,manifest=manifest,
            hashes={str(p.relative_to(run)):sha(p) for p in files}))
    clear=[r for r in rows if r['scenario']=='clear' and r['aided']]
    return dict(root=str(root),stage=stage,planned=len(plan['jobs']),completed=len(rows),
        bootstrap_failures=bootstrap_failures,
        terminal_counts=dict(Counter(r['status']['final_reason'] for r in rows)),
        aided_clear_completed=sum(r['status']['final_phase']=='END' for r in clear),
        aided_clear_trials=len(clear),runs=rows)


def single(root,stage):
    manifest=json.loads((root/'manifest.json').read_text(encoding='utf-8'))
    status=json.loads((root/'run_status.json').read_text(encoding='utf-8'))
    files=[root/n for n in ('manifest.json','run_status.json','mission.jsonl','telemetry.jsonl')]
    files += [root/n for n in status['ulog_files']]
    files += [p for p in (root/'scan_observations.jsonl',) if p.exists()]
    files += sorted((root/'source_snapshot').glob('*'))
    return dict(root=str(root),stage=stage,planned=1,completed=1,
        aided_clear_completed=int(manifest['lidar_aiding'] and status['final_phase']=='END'),
        aided_clear_trials=int(manifest['lidar_aiding'] and manifest['scenario']=='clear'),
        runs=[dict(run=root.name,scenario=manifest['scenario'],seed=manifest['seed'],
            aided=manifest['lidar_aiding'],status=status,manifest=manifest,
            hashes={str(p.relative_to(root)):sha(p) for p in files})])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--development',type=Path,nargs='+',default=[])
    p.add_argument('--development-single',type=Path,nargs='+',default=[])
    p.add_argument('--validation',type=Path,nargs='+',required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    groups=([group(root,'development') for root in args.development]
        +[single(root,'development') for root in args.development_single]
        +[group(root,'validation') for root in args.validation])
    value=dict(schema=1,scope='isolated synthetic SIH; no hardware writes',
        total_trials=sum(g['completed'] for g in groups),groups=groups)
    args.output.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(dict(total_trials=value['total_trials'],groups=[{k:g[k] for k in
        ('root','stage','planned','completed','aided_clear_completed','aided_clear_trials')} for g in groups])))
