"""Read local service status without contacting PX4 or transmitting commands."""
import datetime as dt
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"python"))
from sangwon_web.common import decode
from sangwon_web.ipc import CoreClient, CoreError

def main():
    result={"scope":"HOST_AND_WEB_DIAGNOSTICS","flight_authority":False}
    try:
        result["core"]=CoreClient(ROOT/".runtime/service/core.sock").call("status")
    except (OSError,CoreError) as exc:
        result["core"]={"available":False,"code":str(exc)}
    for key,path in (("web",".runtime/web/adapter_status.json"),("host",".runtime/host_health.json"),("perception",".runtime/perception/health.json"),("px4",".runtime/px4/health.json")):
        try:
            value=decode((ROOT/path).read_bytes())
            if key=="web":
                age=(dt.datetime.now(dt.timezone.utc)-dt.datetime.fromisoformat(value["generated_at"].replace("Z","+00:00"))).total_seconds()
                value["age_s"]=round(age,2);value["fresh"]=0<=age<15
            if key=="perception":
                from sangwon_sensors.perception import host_checks
                import time
                value["host_checks"]=host_checks(value,Path('/proc/sys/kernel/random/boot_id').read_text().strip(),time.monotonic())
            if key=="px4":
                from sangwon_sensors.px4 import host_checks
                import time
                value["host_checks"]=host_checks(value,Path('/proc/sys/kernel/random/boot_id').read_text().strip(),time.monotonic())
            result[key]=value
        except (OSError,ValueError):result[key]={"available":False}
    print(json.dumps(result,ensure_ascii=False,indent=2))

if __name__=="__main__":main()
