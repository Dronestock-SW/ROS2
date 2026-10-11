"""Download a closed PX4 log through the existing MAVROS router, while disarmed.

Only LOG_REQUEST_LIST/DATA/END messages are transmitted. No serial ownership,
flight commands, parameter changes, logger control or file deletion.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time

os.environ['MAVLINK20'] = '1'
from px4_mavros_readback import MavrosReadback


class LogReader(MavrosReadback):
    def receive(self, msg):
        if msg.sysid == 1 and msg.compid == 1 and msg.msgid in (118, 120):
            try:
                self.queue.append(self.parser.decode(self.to_bytes(msg)))
            except Exception:
                pass
        else:
            super().receive(msg)

    def send(self, packet):
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=.01)
            now = time.monotonic()
            if self.state is not None and self.state.armed:
                raise RuntimeError('FC armed; stop log transfer')
            if (self.state is not None and self.heartbeat is not None
                    and self.state.connected and not self.heartbeat.base_mode & 128
                    and now-self.state_at < 1.5 and now-self.heartbeat_at < 1.5
                    and self.publisher.get_subscription_count() == 1):
                break
        else:
            raise RuntimeError('Fresh disarmed FC and one existing router required')
        assert packet.get_type() in ('LOG_REQUEST_LIST', 'LOG_REQUEST_DATA', 'LOG_REQUEST_END')
        packet.pack(self.encoder)
        self.encoder.seq = (self.encoder.seq+1) % 256
        self.publisher.publish(self.to_ros(packet))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--ros-domain', type=int, choices=(1, 2, 99), required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--id', type=int, help='Observed LOG_ENTRY id; default only lists entries')
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    c = LogReader(args.ros_domain, ())
    try:
        assert c.wait_heartbeat(8), 'FC heartbeat missing'
        c.send(c.encoder.log_request_list_encode(1, 1, 0, 65535))
        entries = {};end = time.monotonic()+12
        while time.monotonic() < end:
            m = c.recv_match(blocking=True, timeout=.1)
            if m is not None and m.get_type() == 'LOG_ENTRY':
                entries[m.id] = m.to_dict()
                if len(entries) >= m.num_logs:break
        (args.output/'entries.json').write_text(json.dumps(entries, indent=2), encoding='utf-8')
        print(json.dumps(entries), flush=True)
        if args.id is None:return
        entry = entries[args.id];size = entry['size']
        if not 0 < size <= 32*1024*1024:raise ValueError('Closed log must be 1 byte to 32 MiB')
        data = bytearray(size);received = bytearray(size);done=0
        # Fixed 18 kB windows cap memory and request traffic; retry only gaps.
        for start in range(0, size, 18000):
            finish = min(size, start+18000)
            for attempt in range(8):
                missing = next((i for i in range(start, finish) if not received[i]), None)
                if missing is None:break
                c.send(c.encoder.log_request_data_encode(1, 1, args.id, missing, finish-missing))
                deadline = time.monotonic()+3
                while time.monotonic() < deadline:
                    m = c.recv_match(blocking=True, timeout=.05)
                    if m is None or m.get_type() != 'LOG_DATA' or m.id != args.id:continue
                    lo,hi=m.ofs,min(size,m.ofs+m.count)
                    if lo < 0 or hi < lo or hi-lo > 90:continue
                    done += sum(1 for v in received[lo:hi] if not v)
                    data[lo:hi]=bytes(m.data[:hi-lo]);received[lo:hi]=b'\1'*(hi-lo)
                    if all(received[start:finish]):break
            if not all(received[start:finish]):raise RuntimeError('Log transfer gap at '+str(start))
            if start % 180000 == 0:print(f'{done}/{size} bytes', flush=True)
        assert bytes(data[:7]) == b'ULog\x01\x12\x35'
        path=args.output/f'log-{args.id}.ulg';path.write_bytes(data)
        (args.output/'receipt.json').write_text(json.dumps(dict(id=args.id,bytes=size,
            sha256=hashlib.sha256(data).hexdigest(),flight_commands=False),indent=2),encoding='utf-8')
        print(str(path),flush=True)
    finally:
        try:c.send(c.encoder.log_request_end_encode(1, 1))
        finally:c.close()


if __name__ == '__main__':main()
