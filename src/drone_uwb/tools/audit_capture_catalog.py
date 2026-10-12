"""Audit capture metadata without promoting summaries to flight/fusion proof."""
import argparse
from datetime import datetime, timezone, timedelta
import hashlib
import json
from pathlib import Path

KST = timezone(timedelta(hours=9))


def kst(value):
    return datetime.fromtimestamp(value / 1e9, timezone.utc).astimezone(KST).isoformat()


def audit(metadata):
    records = []
    for capture in metadata:
        manifest = (capture.get('manifest.json') or {}).get('content') or {}
        summary = (capture.get('summary.json') or {}).get('content') or {}
        start, end = manifest.get('started', {}), summary.get('updated', {})
        if not start or not end:
            records.append(dict(path=capture['path'], status='metadata_incomplete'))
            continue
        drift = (end['wall_ns']-start['wall_ns']-(end['monotonic_ns']-start['monotonic_ns']))/1e9
        ground = summary.get('ground_observation', {})
        transitions = summary.get('state_transitions', [])
        candidate = (ground.get('all_ground') is False and any(
            s.get('armed') is True and s.get('mode') == 'POSCTL' for s in transitions))
        records.append(dict(path=capture['path'], boot_id=manifest.get('boot_id'),
            manifest_sha256=capture.get('manifest.json', {}).get('sha256'),
            summary_sha256=capture.get('summary.json', {}).get('sha256'),
            start_kst=kst(start['wall_ns']), end_kst=kst(end['wall_ns']),
            duration_monotonic_s=(end['monotonic_ns']-start['monotonic_ns'])/1e9,
            wall_minus_monotonic_change_s=drift,
            constant_wall_mapping_usable=abs(drift) <= .1,
            state_transitions=transitions, ground_summary=ground,
            manual_position_flight_candidate=candidate,
            simultaneous_arm_airborne_posctl_proven=False,
            sender_jump_gate_off_proven=False, actual_ev_fusion_proven=False,
            scan_count_summary=summary.get('topics', {}).get('/scan', {}).get('received', 0),
            note='Metadata selects candidates; original events and ULog must prove coincident flight and fusion.'))
    return dict(schema=1, scope='capture_metadata_only', count=len(records), records=records)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--metadata', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    raw = args.metadata.read_bytes()
    result = audit(json.loads(raw))
    result.update(source=str(args.metadata.resolve()), source_sha256=hashlib.sha256(raw).hexdigest(),
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(dict(count=result['count'], output=str(args.output)), ensure_ascii=False))


if __name__ == '__main__':
    main()
