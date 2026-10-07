"""Bounded A1/A2/A3 observation capture with read-only FC telemetry."""
import argparse
from collections import Counter
from datetime import datetime, timedelta, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import BatteryState, Imu, Range
from mavros_msgs.msg import State, TimesyncStatus
from std_msgs.msg import String


def clean(value):
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,dict):return {k:clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [clean(v) for v in value]
    return value


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--seconds',type=float,default=60.)
    parser.add_argument('--stage',default='sensor_check')
    parser.add_argument('--reference-xyz',nargs=3,type=float)
    parser.add_argument('--reference-xy',nargs=2,type=float)
    args=parser.parse_args()
    if args.reference_xyz is not None and args.reference_xy is not None:
        parser.error('use either --reference-xyz or --reference-xy')
    reference=args.reference_xyz if args.reference_xyz is not None else (args.reference_xy+[None] if args.reference_xy is not None else None)
    if not 0<args.seconds<=300:raise ValueError('bounded capture requires 0 < seconds <= 300')
    if subprocess.run(['fuser','/dev/uwb','/dev/pixhawk'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0:
        raise RuntimeError('serial already owned; preserve existing capture')
    root=Path('/home/pgyxn/github/ROS2')
    args.output.mkdir(parents=True,exist_ok=False)
    config=root/'src/drone_uwb/config/runtime/uwb_a123_bench.yaml'
    manifest=dict(stage=args.stage,reference_xyz_m=reference,active_anchors=['A1','A2','A3'],
        fc_output_enabled=False,flight_test=False,planned_seconds=args.seconds,
        start_kst=datetime.now(timezone(timedelta(hours=9))).isoformat())
    (args.output/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    (args.output/'config.yaml').write_bytes(config.read_bytes())
    rclpy.init();node=Node('a123_bench_telemetry')
    counts=Counter();last={}
    records=(args.output/'telemetry.jsonl').open('x',encoding='utf-8')
    def receive(topic,msg):
        row=clean(message_to_ordereddict(msg));counts[topic]+=1;last[topic]=row
        records.write(json.dumps(dict(topic=topic,host_monotonic_ns=time.monotonic_ns(),message=row),ensure_ascii=False,allow_nan=False)+'\n')
    topics={'/mavros/state':State,'/mavros/battery':BatteryState,'/mavros/imu/data':Imu,
            '/mavros/downward_0':Range,'/mavros/timesync_status':TimesyncStatus,'/uwb/status':String}
    subs=[node.create_subscription(cls,topic,lambda msg,t=topic:receive(t,msg),qos_profile_sensor_data) for topic,cls in topics.items()]
    procs=[];logs=[]
    commands=[('mavros',['ros2','run','mavros','mavros_node','--ros-args','-r','__ns:=/mavros',
        '--params-file',str(root/'src/drone_uwb/config/runtime/mavros_btf_bench.yaml'),
        '-p','fcu_url:=/dev/pixhawk:921600','-p','tgt_system:=1','-p','tgt_component:=1','-p','fcu_protocol:=v2.0']),
        ('uwb',['ros2','run','drone_uwb','uwb_node','--ros-args','--params-file',str(config),
         '-p','record_directory:='+str(args.output/'uwb'),'-p','stop_after_s:='+str(args.seconds),
         '-r','/uwb_pose:=/uwb/a123_pose'])]
    start=time.monotonic();last_print=0
    try:
        for name,cmd in commands:
            log=(args.output/(name+'.log')).open('x',encoding='utf-8');logs.append(log)
            procs.append(subprocess.Popen(cmd,cwd=root,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        print(json.dumps(dict(event='capture_started',**manifest,output=str(args.output)),ensure_ascii=False),flush=True)
        while time.monotonic()-start<args.seconds+15:
            rclpy.spin_once(node,timeout_sec=.1)
            if procs[0].poll() is not None:raise RuntimeError('MAVROS exited')
            if procs[1].poll() is not None:break
            elapsed=time.monotonic()-start
            if elapsed-last_print>=20:
                last_print=elapsed;records.flush()
                print(json.dumps(dict(elapsed_s=round(elapsed,1),counts=counts),ensure_ascii=False),flush=True)
    finally:
        for process in reversed(procs):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        summary=dict(duration_s=time.monotonic()-start,counts=counts,last=last,exit_codes=[p.returncode for p in procs],
                     fc_output_enabled=False,flight_test=False)
        (args.output/'telemetry_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        records.close()
        for log in logs:log.close()
        node.destroy_node();rclpy.shutdown()
        print(json.dumps(dict(event='capture_finished',output=str(args.output),counts=counts),ensure_ascii=False),flush=True)


if __name__=='__main__':main()
