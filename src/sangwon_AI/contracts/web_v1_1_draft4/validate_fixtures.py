"""Offline consistency checks for documentation fixtures; no flight/network API."""
from pathlib import Path
from datetime import datetime
import copy
import hashlib
import json
import math

ROOT = Path(__file__).resolve().parent
VERSION = '1.1-draft.4'

def read(name):
    return json.loads((ROOT / name).read_text(encoding='utf-8'))

def check(value, message):
    if not value:
        raise ValueError(message)

def ts(value):
    return datetime.fromisoformat(value.replace('Z', '+00:00'))

def validate_tasks(snapshot):
    tasks = snapshot['route_tasks']
    check(len(tasks) > 0, 'empty task list')
    check(len({t['task_id'] for t in tasks}) == len(tasks), 'duplicate task id')
    labels = {l['label_point_id'] for l in snapshot['labels']}
    workspaces = {w['id'] for w in snapshot['scan_workspaces']}
    for i, task in enumerate(tasks):
        check(task['type'] in ('waypoint', 'scan'), 'unsupported task type')
        if task['type'] == 'waypoint':
            check(set(task['position_m']) == {'x', 'y', 'z'}, 'waypoint requires xyz')
            check(all(type(x) in (float, int) and math.isfinite(x) for x in task['position_m'].values()), 'nonfinite xyz')
            check(task['hold_s'] >= 0, 'negative dwell')
            check('yaw_deg' not in task or (type(task['yaw_deg']) in (int, float) and -180 <= task['yaw_deg'] < 180), 'invalid yaw')
        else:
            check('position_m' not in task, 'scan input must not be a vehicle target')
            check(task['label_point_id'] in labels, 'unknown label')
            check(task['workspace_id'] in workspaces, 'unknown workspace')
            anchor = task['approach_anchor']
            if i == 0:
                check(anchor == {'kind': 'LAUNCH'}, 'first scan anchor')
            else:
                check(anchor == {'kind': 'PREVIOUS_TASK_EXIT', 'task_id': tasks[i-1]['task_id']}, 'nonlocal/cyclic anchor')
    for label in snapshot['labels']:
        normal = label['outward_normal_map']
        check(len(normal) == 3 and abs(sum(n*n for n in normal)-1) < 1e-9, 'invalid label normal')
    for workspace in snapshot['scan_workspaces']:
        check('entry_anchor_task_id' not in workspace, 'mission anchor stored in reusable map')

def main():
    manifest = read('manifest.json')
    check(manifest['fixture_only'] and not manifest['physical_flight_authority'], 'fixture authority')
    for entry in manifest['files']:
        data = (ROOT / entry['file']).read_bytes()
        check(len(data) == entry['bytes'], 'file byte length: ' + entry['file'])
        check(hashlib.sha256(data).hexdigest() == entry['sha256'], 'file hash: ' + entry['file'])
        item = json.loads(data)
        if 'contract_version' in item:
            check(item['contract_version'] == VERSION, 'mixed version')
            check(item['drone_id'] == 'TEST-DRONE-01', 'mixed drone')
            check(item['profile'] == 'REPLAY', 'non-REPLAY resource')
        if 'flight_authority' in item:
            check(item['flight_authority'] is False, 'physical authority enabled')
    mission, snapshot = read('mission_full.json'), read('snapshot_content.json')
    blob = (ROOT / 'snapshot_content.json').read_bytes()
    check(mission['snapshot_ref']['sha256'] == hashlib.sha256(blob).hexdigest(), 'snapshot hash mismatch')
    check(mission['snapshot_ref']['byte_length'] == len(blob), 'snapshot size mismatch')
    validate_tasks(snapshot)
    assets = set()
    def collect(obj):
        if isinstance(obj, dict):
            if 'id' in obj and 'revision' in obj and len(obj) > 2:
                assets.add((obj['id'], obj['revision']))
            for val in obj.values(): collect(val)
        elif isinstance(obj, list):
            for val in obj: collect(val)
    collect(snapshot)
    for artifact in snapshot['profile_artifacts']:
        data = (ROOT / manifest['profile_files'][artifact['id']]).read_bytes()
        check(hashlib.sha256(data).hexdigest() == artifact['sha256'], 'profile hash mismatch')
        obj = json.loads(data)
        assets.add((obj['id'], obj['revision']))
        check(obj['approval_status'] == 'SYNTHETIC_ONLY' and obj['valid_for_profiles'] == ['REPLAY'], 'real profile authorization')
    # Label objects use a semantic id instead of generic id.
    for label in snapshot['labels']: assets.add((label['label_point_id'], label['revision']))
    def references(obj):
        if isinstance(obj, dict):
            if set(obj) == {'id', 'revision'}:
                check((obj['id'], obj['revision']) in assets, 'unresolved reference: ' + obj['id'])
            for val in obj.values(): references(val)
        elif isinstance(obj, list):
            for val in obj: references(val)
    references(snapshot)
    for name in ('plan.json', 'launch_snapshot.json'): references(read(name))
    ready, blocked = read('readiness_ready.json'), read('readiness_blocked.json')
    start, accepted, rejected = read('command_start.json'), read('result_accepted.json'), read('result_rejected.json')
    check(ready['context'] == start['context'], 'START readiness context')
    check(len({c['check_id'] for c in ready['checks']}) == 33, 'check catalog incomplete')
    check(not any(c['blocking'] for c in ready['checks']) and ready['can_start'], 'ready blockers')
    check(not blocked['can_start'] and blocked['context']['preparation_id'] is None, 'revocation readiness')
    check(any(c['blocking'] for c in blocked['checks']), 'missing blocker')
    check(ts(ready['generated_at']) <= ts(start['created_at']) < ts(ready['valid_until']), 'stale START example')
    commands = [start] + read('command_active.json')['commands']
    for command in commands:
        check((ts(command['expires_at'])-ts(command['created_at'])).total_seconds() == 10, 'TTL not 10s')
        check(command['context']['snapshot_id'] == snapshot['snapshot_id'], 'command snapshot mismatch')
    check(accepted['control_request_id'] == rejected['control_request_id'] == start['control_request_id'], 'result id mismatch')
    check(accepted['context']['flight_id'] is None, 'flight before takeoff')
    check(rejected['blocking_check_ids'] == ['BP-C13'], 'rejected branch blocker')
    plan, attestation, report = read('plan.json'), read('operator_attestation.json'), read('preparation_report.json')
    check(plan['plan_id'] == attestation['plan_id'] == report['plan_id'], 'plan binding')
    check(report['context'] == ready['context'], 'preparation binding')
    failed, complete = read('scan_task_failed.json'), read('scan_task_completed.json')
    check(failed['task_result_id'] == complete['task_result_id'], 'scan logical id')
    check(failed['result_revision'] < complete['result_revision'], 'scan revision')
    check(complete['scanner_attempts_started'] == 3 and complete['camera_qr_used'], 'scan fallback')
    check(complete['scan_outcome'] == 'FAILED' and complete['egress_status'] == 'COMPLETED', 'scan vs egress')
    check(complete['raw_qr_data'] is None, 'failed scan payload')
    outcome = read('execution_result.json')
    check(outcome['flight_outcome'] == 'SUCCEEDED' and outcome['work_outcome'] == 'PARTIAL_FAILURE', 'result aggregation')
    check(outcome['task_summary']['total'] == len(snapshot['route_tasks']), 'task count')
    check(outcome['task_summary']['failed'] == sum(t['outcome']=='FAILED' for t in outcome['tasks']), 'failed count')
    # Data-level rejection examples; these do not exercise production code.
    mutants = []
    x=copy.deepcopy(snapshot); x['route_tasks'][1]['approach_anchor']['task_id']='S01'; mutants.append(x)
    x=copy.deepcopy(snapshot); x['route_tasks'][1]['position_m']={'x':4.5,'y':1.5,'z':1.2}; mutants.append(x)
    x=copy.deepcopy(snapshot); x['labels'][0]['outward_normal_map']=[0,0,0]; mutants.append(x)
    x=copy.deepcopy(snapshot); x['route_tasks'][0]['position_m'].pop('z'); mutants.append(x)
    x=copy.deepcopy(snapshot); x['route_tasks'][2]['task_id']='W01'; mutants.append(x)
    for mutant in mutants:
        try: validate_tasks(mutant)
        except ValueError: pass
        else: raise ValueError('Invalid fixture was accepted')
    print(f'PASS: {len(manifest["files"])} JSON resources; hashes, references, 33 checks, contexts, TTL, outcomes; 5 invalid data cases rejected.')
    print('Scope: offline document consistency only. No UI/API/flight acceptance tests performed.')

if __name__ == '__main__': main()
