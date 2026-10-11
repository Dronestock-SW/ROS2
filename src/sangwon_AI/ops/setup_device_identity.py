"""Create a Jetson identity once. Never print or export the HMAC secret."""
import argparse
import json
import os
from pathlib import Path
import secrets
import stat
import sys
import uuid

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"python"))
from sangwon_web import CONTRACT_VERSION
from sangwon_web.common import atomic_json, utc_now

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--drone-id",required=True)
    parser.add_argument("--identity-name",default="device")
    parser.add_argument("--profile",choices=("HOST_OBSERVE","REPLAY"),default="HOST_OBSERVE")
    args=parser.parse_args()
    if os.name!="posix":raise SystemExit("Run on the Jetson only")
    if not args.drone_id or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_" for c in args.drone_id):
        raise SystemExit("Invalid drone ID")
    if not args.identity_name or len(args.identity_name)>40 or any(c not in "abcdefghijklmnopqrstuvwxyz0123456789-_" for c in args.identity_name):
        raise SystemExit("Invalid identity name")
    if args.profile=="REPLAY" and not args.drone_id.startswith("TEST-"):
        raise SystemExit("REPLAY requires a separate TEST- drone")
    os.umask(0o077)
    private=ROOT/".runtime/private"
    private.mkdir(parents=True,exist_ok=True,mode=0o700)
    os.chmod(private,0o700)
    path=private/(args.identity_name+".env")
    if not path.exists():
        # O_EXCL avoids accidental replacement/rotation of a registered identity.
        raw=("DRONESTOCK_DEVICE_AUTH_ID=jetson-"+uuid.uuid4().hex+"\n"+
             "DRONESTOCK_DEVICE_AUTH_SECRET="+secrets.token_hex(32)+"\n"+
             "DRONESTOCK_DRONE_ID="+args.drone_id+"\n")
        with os.fdopen(os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600),"w") as file:
            file.write(raw);file.flush();os.fsync(file.fileno())
    if stat.S_IMODE(path.stat().st_mode)!=0o600 or path.is_symlink():
        raise SystemExit("Identity permissions must be 0600, regular file")
    values=dict(line.split("=",1) for line in path.read_text().splitlines())
    if values.get("DRONESTOCK_DRONE_ID")!=args.drone_id:
        raise SystemExit("Existing identity is bound to a different drone; not overwritten")
    if len(values.get("DRONESTOCK_DEVICE_AUTH_SECRET",""))!=64:
        raise SystemExit("Invalid existing secret; not overwritten")
    request={"type":"device_enrollment_request","generated_at":utc_now(),
        "device_auth_id":values["DRONESTOCK_DEVICE_AUTH_ID"],"drone_id":args.drone_id,
        "requested_contract":CONTRACT_VERSION,"requested_auth":"hmac_session_v2",
        "w01_signature_lines":5,"session_signature_lines":7,"requested_profile":args.profile,
        "transport":"Jetson outbound HTTP/WS over Wi-Fi", "registration_state":"REQUESTED_NOT_REGISTERED",
        "secret_included":False,"secret_transfer":"Out-of-band administrator provisioning required",
        "boot_profile":"HOST_OBSERVE","flight_authority":False}
    filename="device_enrollment_request.json" if args.identity_name=="device" else args.identity_name+"_enrollment_request.json"
    atomic_json(ROOT/".runtime"/filename,request)
    print(json.dumps(request,ensure_ascii=False,indent=2))

if __name__=="__main__":main()
