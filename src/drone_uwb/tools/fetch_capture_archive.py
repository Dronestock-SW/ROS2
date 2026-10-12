"""Resume authorized read-only SSH copies and verify original size/SHA-256."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import shlex
import subprocess
import time


def fetch(manifest, output, target, identity):
    for item in json.loads(manifest.read_text(encoding='utf-8')):
        source=PurePosixPath(item['path'])
        if source.name!='events.jsonl' or not source.parent.name.startswith('capture-'):
            raise ValueError('capture file required')
        folder=output/source.parent.name
        folder.mkdir(parents=True,exist_ok=True)
        final=folder/'events.jsonl'
        if final.exists():
            if hashlib.sha256(final.read_bytes()).hexdigest()!=item['sha256']:
                raise ValueError('existing original hash mismatch; preserved')
            continue
        partial=folder/'events.jsonl.ssh-partial'
        offset=partial.stat().st_size if partial.exists() else 0
        if offset>item['bytes']:raise ValueError('oversized partial preserved')
        command='tail -c +'+str(offset+1)+' -- '+shlex.quote(str(source))
        args=['ssh','-C','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes',
            '-o','ConnectTimeout=20','-o','ServerAliveInterval=10','-o','ServerAliveCountMax=3',
            '-i',str(identity),target,command]
        print('RESUME',folder.name,offset,'of',item['bytes'],flush=True)
        with partial.open('ab') as out,(folder/'transfer.stderr').open('ab') as err:
            process=subprocess.Popen(args,stdout=out,stderr=err)
            started=last=time.monotonic()
            while process.poll() is None:
                time.sleep(1)
                if time.monotonic()-last>=30:
                    print('BYTES',folder.name,partial.stat().st_size,'/',item['bytes'],flush=True)
                    last=time.monotonic()
                if time.monotonic()-started>900:
                    process.kill();process.wait()
                    raise RuntimeError('read timeout; partial preserved')
        if process.returncode:raise RuntimeError('SSH read failed; partial and stderr preserved')
        digest=hashlib.sha256(partial.read_bytes()).hexdigest()
        if digest!=item['sha256'] or partial.stat().st_size!=item['bytes']:
            raise RuntimeError('hash or size mismatch; partial preserved')
        partial.rename(final)
        print('VERIFIED',folder.name,item['bytes'],digest,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('manifest','output','identity'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--target',required=True)
    a=p.parse_args();fetch(a.manifest,a.output,a.target,a.identity)
