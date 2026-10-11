"""Bound PC/Jetson UTC offset over verified SSH without changing either clock."""
import argparse
import json
import math
import queue
import re
import subprocess
import threading
import time


def measure(host, count=5):
    if not 1<=count<=8:
        raise ValueError('INVALID_CLOCK_SAMPLE_COUNT')
    if not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@-]*',host):
        raise ValueError('INVALID_SSH_HOST')
    program="import sys,time; print('READY',flush=True); [(print(time.time(),flush=True)) for line in sys.stdin]"
    command=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','StrictHostKeyChecking=yes',host,
             "python3 -u -c '"+program.replace("'","'\"'\"'")+"'"]
    process=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,
                             text=True,encoding='utf-8')
    received=queue.Queue()
    def read_lines():
        for line in process.stdout:
            received.put(line)
        received.put(None)
    threading.Thread(target=read_lines,daemon=True).start()
    def read(timeout):
        try:line=received.get(timeout=timeout)
        except queue.Empty:raise ValueError('CLOCK_PROBE_TIMEOUT') from None
        if line is None:
            raise ValueError('CLOCK_PROBE_UNAVAILABLE')
        return line.strip()
    try:
        if read(10)!='READY':
            raise ValueError('CLOCK_PROBE_UNAVAILABLE')
        samples=[]
        for _ in range(count):
            start=time.time();mono=time.perf_counter()
            process.stdin.write('sample\n');process.stdin.flush()
            remote=float(read(3));end=time.time();rtt=time.perf_counter()-mono
            if not math.isfinite(remote):
                raise ValueError('CLOCK_PROBE_UNAVAILABLE')
            if abs((end-start)-rtt)>.05:
                raise ValueError('CLOCK_CHANGED_DURING_PROBE')
            samples.append((rtt,remote-end,remote-start))
        rtt,low,high=min(samples)
        trusted=rtt<=.1 and low>=-.25 and high<=.25
        return {'scope':'VERIFIED_SSH_UTC_OFFSET_ONLY','state':'PASS' if trusted else 'FAIL',
            'code':'OK' if trusted else ('CLOCK_UNCERTAIN' if rtt>.1 else 'CLOCK_NOT_SYNCHRONIZED'),
            'rtt_ms':round(rtt*1000,2),'remote_ahead_min_ms':round(low*1000,2),
            'remote_ahead_max_ms':round(high*1000,2),'allowed_clock_skew_ms':250,
            'flight_authority':False}
    finally:
        process.stdin.close()
        try:process.wait(timeout=3)
        except subprocess.TimeoutExpired:process.kill();process.wait(timeout=3)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--ssh-host',required=True)
    args=parser.parse_args()
    report=measure(args.ssh_host);print(json.dumps(report))
    return 0 if report['state']=='PASS' else 2


if __name__=='__main__':
    raise SystemExit(main())
