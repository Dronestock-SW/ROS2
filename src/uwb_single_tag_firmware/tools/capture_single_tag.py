"""Bounded direct UART ground capture and schema-2 clock handshake; no FC I/O."""
import argparse
from collections import Counter
import json
from pathlib import Path
import time
import queue
import threading

from drone_uwb.acquisition.serial_io import SerialInput
from drone_uwb.acquisition.framing import LineFramer
from drone_uwb.acquisition.single_tag import SingleTagLink
from drone_uwb.acquisition.validation import InvalidInput
from drone_uwb.contracts.protocol import decode_line, InvalidSample


class AsyncCaptureWriter:
    """Keep SD-card and JSON serialization delays out of the UART read loop."""
    def __init__(self,raw,events):
        self.raw=raw;self.events=events;self.queue=queue.Queue(maxsize=1024)
        self.dropped=0;self.errors=[];self.error_count=0;self.max_write_us=0
        self.worker=threading.Thread(target=self.run,daemon=True)

    def __enter__(self):
        self.worker.start();return self

    def __exit__(self,*unused):
        self.queue.put(None);self.worker.join()

    def put(self,kind,value):
        try:self.queue.put_nowait((kind,value))
        except queue.Full:self.dropped+=1

    def run(self):
        while True:
            item=self.queue.get()
            if item is None:return
            kind,value=item;started=time.monotonic_ns()
            try:
                if kind=='raw':self.raw.write(value)
                else:self.events.write(json.dumps(value,separators=(',',':'),allow_nan=False)+'\n')
            except (OSError,ValueError) as exc:
                self.error_count+=1
                if len(self.errors)<10:self.errors.append(str(exc))
            self.max_write_us=max(self.max_write_us,(time.monotonic_ns()-started)//1000)


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--port',default='/dev/uwb')
    p.add_argument('--seconds',type=float,default=30)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--probe',action='store_true',help='Explicit synthetic pose/parser/recovery bench profile; no FC I/O')
    p.add_argument('--transport-probe',action='store_true',help='Pause host reads for 600ms and reopen the software port; no cable removal')
    p.add_argument('--lora-maxlen-probe',action='store_true',help='Send one 255-byte bench packet for a separately connected ground station')
    args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    counts=Counter(); link=SingleTagLink(); framer=LineFramer(); pending=b''
    commands=[]; first=None; last=None; last_status=None
    latest=None;probe_start=None;stages=set();probe_pose=None;flood_until=0;next_flood=0;last_pose_refresh=0
    serial=SerialInput(args.port,921600,writable=True)
    deadline=time.monotonic()+args.seconds
    previous_loop=None;max_loop_us=0
    try:
        with (args.output/'raw.bin').open('wb') as raw, (args.output/'events.jsonl').open('w',encoding='utf-8') as events, AsyncCaptureWriter(raw,events) as storage:
            def record(row):
                storage.put('event',row)
            while time.monotonic()<deadline:
                now=time.monotonic()
                if previous_loop is not None:max_loop_us=max(max_loop_us,round((now-previous_loop)*1e6))
                previous_loop=now
                if args.lora_maxlen_probe and latest is not None and last_status is not None and link.clock.ready(time.monotonic_ns()//1000):
                    if last_status.get('bench_controls') is not True:
                        raise RuntimeError('Maximum-length probe requires the explicit bench firmware')
                    if 'lora_255' not in stages:
                        stages.add('lora_255')
                        commands.append(link.command('bench',action='lora_255'))
                        record(dict(probe_stage='lora_255',host_monotonic_ns=time.monotonic_ns(),
                                    synthetic_bench=True,fc_output=False,ground_station_reception_verified=False))
                if args.transport_probe and latest is not None and link.clock.ready(time.monotonic_ns()//1000):
                    if probe_start is None:probe_start=now
                    elapsed=now-probe_start
                    if elapsed>=2 and 'host_read_pause_600ms' not in stages:
                        stages.add('host_read_pause_600ms')
                        record(dict(probe_stage='host_read_pause_600ms',host_monotonic_ns=time.monotonic_ns(),
                                    physical_usb_removed=False,fc_output=False))
                        time.sleep(.6)
                    if elapsed>=5 and 'software_port_reopen' not in stages:
                        stages.add('software_port_reopen')
                        record(dict(probe_stage='software_port_reopen',host_monotonic_ns=time.monotonic_ns(),
                                    physical_usb_removed=False,fc_output=False))
                        serial.close();link.disconnect();framer=LineFramer()
                        pending=b'';commands.clear();latest=None
                        serial=SerialInput(args.port,921600,writable=True)
                if args.probe and latest is not None and last_status is not None and link.clock.ready(time.monotonic_ns()//1000):
                    if last_status.get('bench_controls') is not True:
                        raise RuntimeError('Probe requires the explicit bench firmware')
                    if probe_start is None:probe_start=now
                    elapsed=now-probe_start
                    def stage(name):
                        stages.add(name)
                        record(dict(probe_stage=name,host_monotonic_ns=time.monotonic_ns(),synthetic_bench=True,fc_output=False))
                    if elapsed>=2 and 'fresh_pose' not in stages:
                        msg=latest['message'];samples=[x for x in msg['sample_time_us'] if x is not None]
                        result=dict(published=True,ok=True,source_identity=[msg['boot_id'],msg['session']],
                                    host_session=link.host_session,seq=msg['seq'],time_us=max(samples),xy_m=[.125,.25])
                        probe_pose=link.pose_command(result,time.monotonic_ns())
                        probe_pose.update(reason='bench_probe',quality=0)
                        commands.append(probe_pose);last_pose_refresh=now;stage('fresh_pose')
                    if 2<elapsed<3.5 and probe_pose and now-last_pose_refresh>=.15:
                        msg=latest['message'];samples=[x for x in msg['sample_time_us'] if x is not None]
                        result=dict(published=True,ok=True,source_identity=[msg['boot_id'],msg['session']],
                                    host_session=link.host_session,seq=msg['seq'],time_us=max(samples),xy_m=[.125,.25])
                        probe_pose=link.pose_command(result,time.monotonic_ns())
                        probe_pose.update(reason='bench_probe',quality=0)
                        commands.append(probe_pose);last_pose_refresh=now
                    if elapsed>=4 and probe_pose and 'duplicate_expired_pose' not in stages:
                        commands.append(dict(probe_pose));stage('duplicate_expired_pose')
                    if elapsed>=5 and probe_pose and 'reordered_pose' not in stages:
                        commands.append(dict(probe_pose,seq=(probe_pose['seq']-1)&0xffffffff));stage('reordered_pose')
                    if elapsed>=6 and probe_pose and 'old_session_pose' not in stages:
                        commands.append(dict(probe_pose,session=probe_pose['session']^0x55));stage('old_session_pose')
                    if elapsed>=7 and 'malformed_flood' not in stages:
                        commands.extend([b'{"schema":2,"schema":1}\n',b'{"x":NaN}\n',b'x'*4096+b'\n'])
                        flood_until=now+2;stage('malformed_flood')
                    if now<flood_until and now>=next_flood and len(commands)<4:
                        commands.append(b'{"invalid_bench_input":true}\n');next_flood=now+.01
                    if elapsed>=11 and 'uwb_pause' not in stages:
                        commands.append(link.command('bench',action='uwb_pause',duration_ms=3000));stage('uwb_pause')
                    if elapsed>=16 and 'uwb_recover' not in stages:
                        commands.append(link.command('bench',action='uwb_recover'));stage('uwb_recover')
                    if elapsed>=22 and 'lora_recover' not in stages:
                        commands.append(link.command('bench',action='lora_recover'));stage('lora_recover')
                if not pending:
                    if commands:
                        command=commands.pop(0)
                    else:
                        command=link.request(time.monotonic_ns()//1000)
                    if command:
                        if isinstance(command,bytes):
                            pending=command
                            record(dict(direction='host_to_tag',host_monotonic_ns=time.monotonic_ns(),malformed_bytes=len(command)))
                        else:
                            pending=(json.dumps(command,separators=(',',':'))+'\n').encode()
                            record(dict(direction='host_to_tag',host_monotonic_ns=time.monotonic_ns(),message=command))
                if pending:
                    written=serial.write(pending);pending=pending[written:]
                chunk=serial.read()
                if chunk:
                    mono,ros=time.monotonic_ns(),time.time_ns()
                    storage.put('raw',chunk)
                    for line in framer.feed(chunk):
                        try:
                            msg=decode_line(line)
                            kind=msg.get('type','unknown');counts[kind]+=1
                            if kind=='uwb_raw_cycle':
                                if first is None:first=mono
                                last=mono
                                counts['mask_'+str(msg.get('valid_mask'))]+=1
                            if kind=='uwb_raw_status':last_status=msg
                            event,command=link.ingest(msg,mono,ros)
                            if event['single_tag']['mapped']:
                                counts['mapped_raw']+=1
                                if msg.get('valid_mask')==15:latest=event
                            if command:commands.append(command)
                            record(event)
                        except (InvalidSample,InvalidInput,KeyError,TypeError,ValueError) as exc:
                            counts['reject_'+str(exc)]+=1
                            record(dict(rejected=str(exc),host_monotonic_ns=mono,
                                        line=line.decode('utf-8',errors='replace')))
                time.sleep(.001)
    finally:
        serial.close()
        result=dict(seconds=args.seconds,counts=dict(counts),status=last_status,
                    synthetic_bench_probe=args.probe or args.lora_maxlen_probe,
                    synthetic_pose_probe=args.probe,lora_maxlen_probe=args.lora_maxlen_probe,
                    transport_probe=args.transport_probe,probe_stages=sorted(stages),
                    recording_queue_dropped=storage.dropped,recording_errors=storage.errors,
                    recording_error_count=storage.error_count,storage_max_write_us=storage.max_write_us,
                    host_loop_max_interval_us=max_loop_us,
                    clock=link.clock.diagnostics(time.monotonic_ns()//1000),
                    raw_rate_hz=(counts['uwb_raw_cycle']-1)/((last-first)/1e9) if first and last>first else None)
        (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(result),flush=True)


if __name__=='__main__':main()
