"""Explicit ground trial/final flashing with portable address/hash manifest.

prepare checks the live FC, stops the sole receiver and holds Tag B in ROM.
write never changes eFuses; --erase is reserved for the qualified final release.
"""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess

from backup_tag import ground_snapshot, SERVICE, EXPECTED_MAC


def main():
    p=argparse.ArgumentParser()
    p.add_argument('action',choices=['prepare','write'])
    p.add_argument('--images',type=Path)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--erase',action='store_true')
    args=p.parse_args()
    os.umask(0o077)
    args.output.mkdir(parents=True,exist_ok=True)
    root=Path.home()/'esp32-single-tag-20261015'
    python=root/'venv/bin/python'
    def run(command,name,timeout=90):
        result=subprocess.run(list(map(str,command)),capture_output=True,text=True,timeout=timeout)
        text=result.stdout+result.stderr
        (args.output/name).write_text(text,encoding='utf-8')
        if result.returncode:raise RuntimeError(name+': '+text[-1500:])
        return text
    if os.path.realpath('/dev/uwb')!='/dev/ttyUSB1':
        raise RuntimeError('Reinspect changed serial port before any reset')
    base=[python,'-m','esptool','--chip','esp32s3','--port','/dev/uwb','--connect-attempts','2']
    if args.action=='prepare':
        ground=ground_snapshot()
        (args.output/'ground.json').write_text(json.dumps(ground,indent=2)+'\n',encoding='utf-8')
        run(['systemd-run','--user','--unit=esp32-trial-recovery','--on-active=15m',
             '/usr/bin/systemctl','--user','start',SERVICE],'recovery-timer.txt')
        run(['systemctl','--user','stop',SERVICE],'service-stop.txt')
        info=run(base+['--after','no_reset','--no-stub','flash_id'],'identity.txt')
        if EXPECTED_MAC not in info.lower() or '8MB' not in info:
            raise RuntimeError('Unexpected device identity')
        print('PREPARED: fresh disarmed ground, receiver stopped, intended Tag B held in ROM',flush=True)
        return
    if not args.images:raise RuntimeError('--images required')
    manifest=json.loads((args.images/'images.json').read_text(encoding='utf-8'))
    pairs=[]
    for item in manifest['images']:
        image=args.images/item['path']
        if hashlib.sha256(image.read_bytes()).hexdigest()!=item['sha256']:
            raise RuntimeError('Image hash mismatch: '+str(image))
        pairs.extend([item['offset'],image])
    info=run(base+['--after','no_reset','--no-stub','flash_id'],'identity-write.txt')
    if EXPECTED_MAC not in info.lower() or '8MB' not in info:raise RuntimeError('Unexpected Tag B')
    security=run(base+['--after','no_reset','--no-stub','get_security_info'],'security-write.txt')
    if 'Secure Boot: Disabled' not in security or 'Flash Encryption: Disabled' not in security:
        raise RuntimeError('Security state changed')
    if args.erase:
        flash=args.output/'full-flash-pre-erase.bin'
        run(base+['--after','no_reset','--baud','921600','read_flash','--no-progress','0','ALL',flash],
            'pre-erase-backup.txt',360)
        if flash.stat().st_size!=8388608:raise RuntimeError('Incomplete pre-erase backup')
        (args.output/'pre-erase-sha256.txt').write_text(hashlib.sha256(flash.read_bytes()).hexdigest()+'\n',encoding='utf-8')
        run(base+['--after','no_reset','erase_flash'],'erase-flash.txt',120)
    run(base+['--after','no_reset','--baud','921600','write_flash','--flash_mode',manifest['flash_mode'],
              '--flash_freq',manifest['flash_frequency'],'--flash_size',manifest['flash_size'],*pairs],
        'write-flash.txt',180)
    verified=run(base+['--baud','921600','verify_flash',*pairs],'verify-flash.txt',180)
    if verified.lower().count('verify ok (digest matched)') != len(manifest['images']):
        raise RuntimeError('Missing per-image verify success')
    (args.output/'result.json').write_text(json.dumps(dict(manifest=manifest,erase=args.erase,
        verified=True,utc=datetime.datetime.now(datetime.timezone.utc).isoformat()),indent=2)+'\n',encoding='utf-8')
    print('VERIFIED: bootloader, partitions, boot_app0 and application; Tag B reset to run',flush=True)


if __name__=='__main__':main()
