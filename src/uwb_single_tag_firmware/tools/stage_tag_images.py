"""Copy only built image addresses/hashes to a portable release directory."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--tag',type=Path,required=True)
    p.add_argument('--environment',required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    built=args.tag/'.pio/build'/args.environment
    layout=json.loads((built/'flash_layout_export.json').read_text(encoding='utf-8'))
    args.output.mkdir(parents=True,exist_ok=False)
    for image in layout['images']:
        source=Path(image['path'])
        if not source.is_absolute():source=args.tag/source
        target=args.output/source.name
        shutil.copyfile(source,target)
        image.update(path=source.name,sha256=hashlib.sha256(target.read_bytes()).hexdigest(),bytes=target.stat().st_size)
    layout['expected_mac']='e8:3d:c1:e3:48:2c'
    (args.output/'images.json').write_text(json.dumps(layout,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(layout))


if __name__=='__main__':main()
