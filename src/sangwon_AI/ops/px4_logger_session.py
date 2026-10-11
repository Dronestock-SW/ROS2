"""Explicit ground-only ULog session control through existing MAVROS.

Only logger on/off and fixed diagnostic queries are allowed. No FC parameters,
ARM, mode, flight commands, serial ownership changes or automatic boot action.
"""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import time

from px4_mavros_readback import MavrosReadback
from px4_sensor_readback import completed

READ_QUERIES = ('logger status', 'df', 'param show SDLOG_MODE', 'param show SDLOG_PROFILE')
COMMANDS = (*READ_QUERIES, 'logger on', 'logger off')


def parse_logger(text):
    running = 'Full File Logging Running:' in text
    path = re.search(r'Log file: (/fs/microsd/log/[^\s]+\.ulg)', text)
    count = re.search(r'Wrote\s+([0-9.]+)\s+(KiB|MiB)', text)
    return dict(logging=running, path=path.group(1) if path else None,
                written_bytes_approx=(float(count.group(1))*(1024 if count.group(2)=='KiB' else 1048576)
                                      if count else None))


def sd_free_bytes(text):
    match = re.search(r'^\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+/fs/microsd\s*$', text, re.M)
    if not match:raise RuntimeError('SD capacity response missing')
    return int(match.group(1))*int(match.group(4))


def fresh_ground(message, now_ns):
    if message is None:return False
    stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
    return message.landed_state==1 and 0<=now_ns-stamp<=1_500_000_000


class LoggerSession(MavrosReadback):
    def __init__(self, domain):
        super().__init__(domain, COMMANDS)
        from mavros_msgs.msg import ExtendedState
        from rclpy.qos import qos_profile_sensor_data
        self.landed = None
        self.node.create_subscription(ExtendedState, '/mavros/extended_state', self.receive_landed,
                                      qos_profile_sensor_data)

    def receive_landed(self, message):
        self.landed = message

    def query(self, command):
        if command not in COMMANDS:raise RuntimeError('logger command not allowed')
        while self.recv_match(blocking=False):pass
        if not fresh_ground(self.landed, self.node.get_clock().now().nanoseconds):
            raise RuntimeError('fresh ON_GROUND required; no in-flight logger control')
        encoded=(command+'\n').encode('ascii')
        # The inherited transport additionally checks fresh DISARM, heartbeat
        # and exactly one existing router before every outgoing shell packet.
        self.serial_control_send(10,6,0,0,len(encoded),list(encoded)+[0]*(70-len(encoded)))
        text='';end=time.monotonic()+5
        while time.monotonic()<end:
            packet=self.recv_match(blocking=True,timeout=.1)
            if packet is None:continue
            if packet.get_type()=='HEARTBEAT' and packet.base_mode & 128:
                raise RuntimeError('FC armed; stop session operations')
            if packet.get_type()=='SERIAL_CONTROL':
                text+=bytes(packet.data[:packet.count]).decode('utf-8',errors='replace')
                if completed(text,command):return text
        raise RuntimeError('Incomplete logger response: '+command)

    def wait(self, seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:self.recv_match(blocking=True,timeout=.1)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    action=p.add_mutually_exclusive_group()
    action.add_argument('--start',action='store_true')
    action.add_argument('--stop',action='store_true')
    p.add_argument('--ros-domain',type=int,choices=(1,2,99),required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();os.environ['MAVLINK20']='1'
    report=dict(utc=datetime.now(timezone.utc).isoformat(),ros_domain=args.ros_domain,
                parameter_writes=False,flight_commands=False,queries=[],growth_verified=False)
    session=LoggerSession(args.ros_domain)
    def query(command):
        response=session.query(command)
        report['queries'].append(dict(command=command,response=response,monotonic_ns=time.monotonic_ns()))
        return response
    try:
        if session.wait_heartbeat(8) is None:raise RuntimeError('FC heartbeat unavailable')
        session.wait(.5)
        free=sd_free_bytes(query('df'));report['sd_free_bytes']=free
        for q in READ_QUERIES[2:]:query(q)
        before=parse_logger(query('logger status'));report['before']=before
        if args.start and not before['logging']:
            if free<512*1024**2:raise RuntimeError('Back up SD logs first; less than 512MiB free')
            query('logger on');session.wait(2)
        if args.stop:
            query('logger off');session.wait(4)
        first=parse_logger(query('logger status'));report['first']=first
        if args.stop and first['logging']:
            raise RuntimeError('ULog still running; stop not confirmed')
        if args.start:
            session.wait(5)
            last=parse_logger(query('logger status'));report['last']=last
            report['growth_verified']=(first['logging'] and last['logging']
                and first['path'] is not None and first['path']==last['path']
                and first['written_bytes_approx'] is not None and last['written_bytes_approx'] is not None
                and last['written_bytes_approx']>first['written_bytes_approx'])
            if not report['growth_verified']:raise RuntimeError('ULog growth not established')
        for q in READ_QUERIES[2:]:query(q)
    except Exception as exc:
        report['error']=str(exc)
        raise
    finally:
        # Release only the shell. Logging intentionally continues after --start.
        try:session.serial_control_send(10,0,0,0,0,[0]*70)
        finally:
            session.close();args.output.parent.mkdir(parents=True,exist_ok=True)
            args.output.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='queries'}))


if __name__=='__main__':main()
