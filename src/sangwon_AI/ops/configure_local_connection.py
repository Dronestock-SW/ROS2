"""Configure boot-time HOST_OBSERVE authentication to the user's local web server."""
import argparse
import json
import os
from pathlib import Path
import sys
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'python'))
from sangwon_web.common import atomic_json
from sangwon_web.identity import load_identity


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--server', required=True, help='http(s)://PC-LAN-IP:port')
    parser.add_argument('--drone-id', required=True, help='Existing web aircraft ID, not the tag alias')
    args = parser.parse_args()
    server = args.server.rstrip('/')
    origin = urlsplit(server)
    if (origin.scheme not in ('http', 'https') or not origin.hostname or origin.path or origin.query
            or origin.fragment or origin.username or origin.password):
        raise SystemExit('An HTTP(S) server origin is required')
    identity = load_identity(ROOT/'.runtime/private/device.env', args.drone_id)
    config_path = ROOT/'config/companion.local.json'
    core = json.loads((config_path if config_path.exists() else ROOT/'config/companion.host.json').read_text())
    if core['profile'] != 'HOST_OBSERVE' or core['physical_output_enabled'] is not False:
        raise SystemExit('Only HOST_OBSERVE local connection setup is supported')
    if (ROOT/'.runtime/service/ledger.sqlite3').exists() and core['drone_id'] != args.drone_id:
        raise SystemExit('Existing runtime belongs to another aircraft; use a separate package instance')
    core.update(drone_id=args.drone_id)
    web = {'mode': 'contract', 'integration_stage': 'full', 'drone_id': args.drone_id,
        'base_url': server, 'core_socket': '.runtime/service/core.sock', 'state_dir': '.runtime/web',
        'poll_interval_s': .5, 'timeout_s': 3, 'auth': {'mode': 'hmac_session_v2'},
        'websocket_url': server.replace('https://', 'wss://').replace('http://', 'ws://') + '/ws/drones/' + args.drone_id + '/'}
    os.umask(0o077)
    atomic_json(ROOT/'config/companion.local.json', core)
    atomic_json(ROOT/'config/web.local.json', web)
    print(json.dumps({'configured': True, 'server': server, 'drone_id': args.drone_id,
        'device_id': identity['DRONESTOCK_DEVICE_AUTH_ID'], 'profile': 'HOST_OBSERVE',
        'restart_required': True, 'server_registration_required': True, 'flight_authority': False}))


if __name__ == '__main__':
    main()
