"""Fetch web-team commit metadata without changing its checkout or running its code."""
import datetime as dt
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
from sangwon_web.common import atomic_json

REPO=ROOT/'.runtime/web-source'
EXPECTED='https://github.com/ljh006008-blip/https---github.com-ljh006008-blip-Teamproject1'
STATE=ROOT/'.runtime/web_monitor/git_latest.json'

def git(*args):
    env=dict(os.environ,GIT_TERMINAL_PROMPT='0',GCM_INTERACTIVE='Never')
    result=subprocess.run(['git','-C',str(REPO),'-c','credential.interactive=never',*args],
                          capture_output=True,text=True,encoding='utf-8',errors='replace',env=env,timeout=45)
    if result.returncode:
        # Never preserve credential-helper output or a URL containing a credential.
        raise RuntimeError('GIT_COMMAND_FAILED')
    return result.stdout.strip()

def main():
    previous=json.loads(STATE.read_text(encoding='utf-8')) if STATE.exists() else {}
    result={'checked_at':dt.datetime.now(dt.timezone.utc).isoformat(),'remote_read_only':True,
            'checkout_modified':False,'notify':False,'last_good':previous.get('last_good')}
    try:
        if git('remote','get-url','origin').removesuffix('.git')!=EXPECTED:
            raise RuntimeError('UNEXPECTED_REMOTE')
        git('fetch','--no-tags','--depth=40','origin','main')
        current=git('rev-parse','refs/remotes/origin/main')
        old=previous.get('last_good')
        changed=bool(old and old!=current)
        recovered=previous.get('consecutive_failures',0)>=3
        result.update(last_good=current,previous_commit=old,changed=changed,recovered=recovered,
                      consecutive_failures=0,notify=changed or recovered)
        if changed:
            # Paths/commit summaries only; raw source/config/mission content stays out of monitoring state.
            result['changed_files']=git('diff','--name-only',old,current,'--','dashboard/DroneStock-main','docs').splitlines()
            result['commits']=git('log','--max-count=12','--format=%h %aI %an %s',old+'..'+current).splitlines()
        else:
            result['commits']=git('log','-1','--format=%h %aI %an %s',current).splitlines()
    except (RuntimeError,OSError,subprocess.TimeoutExpired) as exc:
        failures=previous.get('consecutive_failures',0)+1
        result.update(consecutive_failures=failures,notify=failures==3,
                      code=str(exc) if isinstance(exc,RuntimeError) else type(exc).__name__)
    atomic_json(STATE,result)
    print(json.dumps(result,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
