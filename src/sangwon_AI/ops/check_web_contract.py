"""Read-only web contract probe. Stores schema/status only, never mission values."""
import datetime as dt
import json
from pathlib import Path
import sys
import urllib.error
import urllib.parse
import urllib.request

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"python"))
from sangwon_web.common import atomic_json, decode
from sangwon_web.transport import Transport

class ProbeNoRedirect(urllib.request.HTTPRedirectHandler):
    """Expose redirect status without visiting login or another origin."""
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def probe():
    transport=Transport(dict(mode="observe",base_url="http://203.247.41.82:8876",timeout_s=6,auth=dict(mode="none")))
    transport.opener=urllib.request.build_opener(ProbeNoRedirect())
    results={}
    for path in ("/api/drones/5/companion-mission/","/api/drones/5/autonomy-state/"):
        try:
            data=transport.request("GET",path)
            if not isinstance(data,dict):raise ValueError("EXPECTED_JSON_OBJECT")
            results[path]={"http_status":200,"contract_version":data.get("contract_version"),"fields":sorted(data)}
        except urllib.error.HTTPError as exc:
            results[path]={"http_status":exc.code}
            # Do not persist redirect URLs/query strings or response bodies.
            if exc.code in (301,302,303,307,308):
                target=urllib.parse.urlsplit(exc.headers.get("Location", ""))
                login=target.path.rstrip("/")=="/platform/login" and (
                    not target.netloc or target.netloc==transport.origin.netloc)
                results[path]["response_state"]="AUTH_REQUIRED" if login else "REDIRECT_REFUSED"
            elif exc.code in (401,403):
                results[path]["response_state"]="AUTH_REQUIRED"
            exc.close()
        except (OSError,ValueError) as exc:
            results[path]={"error_type":type(exc).__name__}
    return results

def main():
    path=ROOT/".runtime/web_monitor/latest.json"
    previous=decode(path.read_bytes()) if path.exists() else {}
    current=probe()
    failed=any("error_type" in value or value.get("http_status",0)>=500 for value in current.values())
    failures=previous.get("consecutive_failures",0)+1 if failed else 0
    last_good=previous.get("last_good",{})
    changed=bool(last_good) and not failed and current!=last_good
    recovered=not failed and previous.get("consecutive_failures",0)>=3
    result={"checked_at":dt.datetime.now(dt.timezone.utc).isoformat(),"read_only":True,
        "checks":current,"changed":changed,"previous_checks":last_good if changed else None,
        "consecutive_failures":failures,"last_good":last_good if failed else current,
        "notify":changed or failures==3 or recovered,"recovered":recovered}
    history=previous.get("history",[])
    if not history and previous.get("checked_at"):
        history=[{"checked_at":previous["checked_at"],"changed":previous.get("changed",False),
                  "checks":{key:{k:v for k,v in item.items() if k!="fields"}
                            for key,item in previous.get("checks",{}).items()}}]
    history.append({"checked_at":result["checked_at"],"changed":changed,
                    "checks":{key:{k:v for k,v in item.items() if k!="fields"} for key,item in current.items()}})
    result["history"]=history[-72:]
    result["last_change_at"]=result["checked_at"] if changed else previous.get("last_change_at")
    atomic_json(path,result)
    print(json.dumps(result,ensure_ascii=False,indent=2))

if __name__=="__main__":main()
