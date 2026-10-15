"""Verify every Intel HEX byte against an independently read 512 KiB flash."""
import argparse
import hashlib
import json
from pathlib import Path
from intelhex import IntelHex


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--hex',type=Path,required=True)
    parser.add_argument('--readback',type=Path,required=True)
    parser.add_argument('--anchor',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    flash=args.readback.read_bytes()
    if len(flash)!=512*1024:raise RuntimeError('Incomplete nRF52832 flash readback')
    source=IntelHex(str(args.hex))
    addresses=source.addresses()
    if any(address>=len(flash) for address in addresses):
        raise RuntimeError('Image has non-flash segments; verify those separately')
    differences=[address for address in addresses if flash[address]!=source[address]]
    if differences:raise RuntimeError('Readback mismatch at '+hex(differences[0]))
    result=dict(anchor=args.anchor,image_sha256=hashlib.sha256(args.hex.read_bytes()).hexdigest(),
        flash_readback_sha256=hashlib.sha256(flash).hexdigest(),compared_image_bytes=len(addresses),
        other_segments=[],verified=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result))


if __name__=='__main__':main()
