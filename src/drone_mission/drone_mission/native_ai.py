"""Bind the existing C++ geometry checks to native mission execution."""
import copy
import hashlib
import json
import math
from pathlib import Path
import subprocess
from .site import ceiling_map
from .contracts import finite


class NativePlanner:
    def __init__(self, binary, map_file, expected_sha256, *, allow_virtual=False):
        self.binary = str(Path(binary).resolve())
        raw = Path(map_file).read_bytes()
        self.raw_map = raw
        if (len(expected_sha256) != 64 or hashlib.sha256(raw).hexdigest() != expected_sha256):
            raise ValueError('native_map_hash_mismatch')
        self.map = json.loads(raw)
        if (self.map.get('survey_status') != 'SURVEYED'
                and not (allow_virtual and self.map.get('survey_status') == 'VIRTUAL')):
            raise ValueError('field_map_survey_required')
        self.map_sha256 = expected_sha256
        self.cache = {}

    @staticmethod
    def key(payload):
        route = {k: v for k, v in payload.items() if k not in (
            'control_action', 'control_request_id', 'control_requested_at', 'generated_at')}
        return hashlib.sha256(json.dumps(route, sort_keys=True, allow_nan=False).encode()).hexdigest()

    def reuse(self, payload):
        cached = self.cache.get(self.key(payload))
        if cached is None:
            raise ValueError('native_plan_route_not_prepared')
        tasks, proof = copy.deepcopy(cached)
        return dict(payload, route_tasks=tasks), proof

    def compile(self, payload, launch_xy, settings):
        planned = payload.get('planned_launch_xy_m')
        if planned is not None and (not isinstance(planned,list) or len(planned)!=2
                or not finite(*planned) or math.dist(planned,launch_xy) > .1):
            raise ValueError('automatic_launch_position_changed_regenerate_trial')
        key = self.key(payload)
        if key in self.cache:
            return self.reuse(payload)
        site_map = (ceiling_map(self.map, payload['ceiling_height_m'])
                    if 'ceiling_height_m' in payload else self.map)
        source = dict(schema='sangwon-native-plan/1', map=site_map, assignment=payload,
                      mag_type=settings.expected_ekf2_mag_type,
                      native_height_m=settings.expected_mis_takeoff_alt_m,
                      max_leg_m=settings.max_leg_m, launch_xy_m=list(launch_xy))
        result = subprocess.run([self.binary], input=json.dumps(source, allow_nan=False),
                                capture_output=True, text=True, timeout=.5, check=False)
        if result.returncode:
            raise ValueError('native_plan_rejected:'+result.stderr.strip()[:256])
        plan = json.loads(result.stdout)
        if (plan.get('schema') != source['schema'] or plan.get('validated') is not True
                or plan.get('flight_authority') is not False):
            raise ValueError('native_plan_invalid_reply')
        tasks = plan['route_tasks']
        original = payload['route_tasks']
        if len(tasks) != len(original):
            raise ValueError('native_plan_task_mismatch')
        for old, new in zip(original, tasks):
            a, b = dict(old), dict(new)
            a.pop('path_validation_ref', None)
            b.pop('path_validation_ref', None)
            if a != b:
                raise ValueError('native_plan_changed_mission')
        proof = dict(plan_ref=plan['plan_ref'], map_sha256=self.map_sha256,
                     launch_xy_m=list(launch_xy), scope=plan['scope'],
                     ceiling_height_m=payload.get('ceiling_height_m'))
        # One process executes at most one mission. Bound rejected/pending requests too.
        if len(self.cache) >= 20:
            raise ValueError('native_plan_request_budget_exceeded')
        self.cache[key] = (tasks, proof)
        return self.reuse(payload)
