"""Two real C++/Python REPLAY instances against a separately running Django peer."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'python'))
sys.path.insert(0, str(ROOT / 'tests'))
from sangwon_web.ipc import CoreClient, CoreError
from replay_scan_fixture import bundle

DRONES = ('TEST-DRONE-01', 'TEST-DRONE-02')
# Only the explicitly isolated Django test settings register these fixture keys.
FIXTURE_KEYS = {'companion-a': 'isolated-unit-a', 'companion-b': 'isolated-unit-b'}
FIXTURE_DEVICES = dict(zip(FIXTURE_KEYS, DRONES))
HOST_DRONE = 'FIELD-OBSERVE-01'
HOST_DEVICE = 'companion-host-fixture'
FIXTURE_KEYS[HOST_DEVICE] = 'isolated-host-key'


def snapshot_bytes(drone, scan=False):
    if scan:
        return bundle(drone)[0]
    value = json.loads((ROOT / 'contracts/runtime_replay/waypoint_snapshot.json').read_bytes())
    value.update(drone_id=drone, snapshot_id='django-peer-' + drone)
    return json.dumps(value, sort_keys=True, separators=(',', ':')).encode()


def stop(process):
    if process and process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)


class Peer:
    def __init__(self, root, drone, device, daemon, url, profile='REPLAY', scan=False):
        self.root, self.drone = root, drone
        (root / 'config').mkdir(parents=True)
        core = {'profile': profile, 'drone_id': drone, 'physical_output_enabled': False,
                'state_dir': '.runtime/c', 'replay_steps_per_tick': 5,
                'approved_replay_snapshot_sha256': [hashlib.sha256(snapshot_bytes(drone)).hexdigest()]}
        if scan and profile=='REPLAY':
            core.update(bundle(drone, 'CAMERA_SUCCESS' if drone==DRONES[0] else 'QR_UNREADABLE')[1])
            core['replay_steps_per_tick']=10
        web = {'mode': 'contract', 'integration_stage': 'full', 'drone_id': drone,
            'base_url': url, 'websocket_url': url.replace('http://', 'ws://') + '/ws/drones/' + drone + '/',
            'core_socket': '.runtime/c/core.sock', 'state_dir': '.runtime/w',
            'poll_interval_s': 0.2, 'timeout_s': 3, 'auth': {'mode': 'hmac_session_v2'}}
        for name, config in [('core', core), ('web', web)]:
            (root / 'config' / (name + '.json')).write_text(json.dumps(config))
        self.env = dict(os.environ, PYTHONPATH=str(ROOT / 'python'), PYTHONDONTWRITEBYTECODE='1',
            DRONESTOCK_DEVICE_AUTH_ID=device, DRONESTOCK_DEVICE_AUTH_SECRET=FIXTURE_KEYS[device])
        self.handles, self.processes = [], []
        if profile == 'HOST_OBSERVE':
            self.launch('health', [sys.executable, str(ROOT/'ops/health_monitor.py'), '--root', str(root)])
        self.core_process = self.launch('core', [str(daemon), '--config', str(root / 'config/core.json')])
        self.client = CoreClient(root / '.runtime/c/core.sock')
        self.adapter = None
        self.start_adapter()

    def launch(self, name, args):
        handle = (self.root / (name + '.log')).open('ab')
        self.handles.append(handle)
        process = subprocess.Popen(args, env=self.env, stdout=handle, stderr=subprocess.STDOUT)
        self.processes.append(process)
        return process

    def start_adapter(self):
        self.adapter = self.launch('web', [sys.executable, '-m', 'sangwon_web.adapter',
                                         '--config', str(self.root / 'config/web.json')])

    def close(self):
        for process in reversed(self.processes):
            stop(process)
        for handle in self.handles:
            handle.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--daemon', required=True)
    parser.add_argument('--base-url', required=True)
    parser.add_argument('--timeout', type=float, default=75)
    parser.add_argument('--scan', action='store_true')
    args = parser.parse_args()
    if not args.base_url.startswith('http://127.0.0.1:'):
        raise ValueError('ISOLATED_LOOPBACK_PEER_REQUIRED')
    daemon = Path(args.daemon).resolve()
    def interrupted(*_):
        raise SystemExit('Integration interrupted')
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    peers = []
    with tempfile.TemporaryDirectory(prefix='sj-django-') as directory:
        try:
            for index, (device, drone) in enumerate(FIXTURE_DEVICES.items()):
                peers.append(Peer(Path(directory) / str(index), drone, device, daemon, args.base_url, scan=args.scan))
            peers.append(Peer(Path(directory)/'host', HOST_DRONE, HOST_DEVICE, daemon, args.base_url, 'HOST_OBSERVE'))
            interrupted_link, restarted = False, False
            deadline = time.monotonic() + args.timeout
            next_diagnostic = time.monotonic() + 3
            while time.monotonic() < deadline:
                states = []
                try:
                    states = [p.client.call('status') for p in peers]
                except (OSError, CoreError):
                    time.sleep(.1)
                    continue
                if time.monotonic() >= next_diagnostic:
                    diagnostics = []
                    for p in peers:
                        try:
                            value = json.loads((p.root / '.runtime/w/adapter_status.json').read_text())
                            diagnostics.append({'drone_id': p.drone, 'state': value['state'], 'code': value['code']})
                        except (OSError, ValueError):
                            pass
                    print(json.dumps({'progress': diagnostics}), flush=True)
                    next_diagnostic = time.monotonic() + 3
                if not interrupted_link and states[0]['telemetry']['phase'] == ('SCANNING' if args.scan else 'NAVIGATING'):
                    stop(peers[0].adapter)
                    peers[0].client.call('link.update', {'connected': False, 'code': 'TEST_WIFI_DISCONNECT',
                                                       'observed_contract': '1.1-draft.4'})
                    interrupted_link = True
                if interrupted_link and not restarted and states[0]['telemetry']['phase'] == 'ENDED':
                    peers[0].start_adapter()
                    restarted = True
                if restarted and all(s['telemetry']['phase'] == 'ENDED' for s in states[:2]):
                    if all(not p.client.call('outbox.list') for p in peers):
                        visits=3 if args.scan else 2
                        assert all(s['telemetry']['visited'] == visits for s in states[:2])
                        assert all(s['physical_output_enabled'] is False for s in states)
                        assert states[2]['profile'] == 'HOST_OBSERVE' and not states[2]['readiness']['can_start']
                        assert states[2]['readiness']['host_diagnostics']['state'] == 'LIVE'
                        assert states[2]['telemetry']['px4']['connected'] is False
                        print(json.dumps({'passed': True, 'profile': 'REPLAY', 'flight_authority': False,
                            'drone_ids': list(DRONES), 'independent_runtime_ids': len({s['context']['runtime_session_id'] for s in states}) == 3,
                            'host_authenticated_diagnostics': True, 'host_has_no_flight_authority': True,
                            'adapter_disconnect_during_mission': interrupted_link,
                            'reconnected_outbox_drained': True, 'scan_fixture': args.scan,
                            'visited_per_drone': [visits, visits]}), flush=True)
                        return
                time.sleep(.1)
            diagnostics = []
            for peer in peers:
                state = peer.root / '.runtime/w/adapter_status.json'
                try:
                    value = json.loads(state.read_text())
                    diagnostics.append({'drone_id': peer.drone, 'state': value['state'], 'code': value['code']})
                except (OSError, ValueError):
                    diagnostics.append({'drone_id': peer.drone, 'state': 'NOT_REPORTED'})
            raise AssertionError('PEER_TIMEOUT ' + json.dumps(diagnostics))
        finally:
            for peer in reversed(peers):
                peer.close()


if __name__ == '__main__':
    main()
