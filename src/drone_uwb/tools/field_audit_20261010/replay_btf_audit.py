"""File-only replay using the production BtfNode callbacks; never initializes ROS."""
import os
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
import collections
import gzip
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
from types import SimpleNamespace
import time

sys.path.insert(0, '/home/arialhanho/ROS2-review-20261008-codex/src/drone_uwb')
from drone_uwb.integration.ros import btf_node as production
from drone_uwb.processing.measured_btf import MeasuredBtf
from drone_uwb.integration.clock_readiness import ClockReadiness

SNAPSHOT = Path('/tmp/btf-history-audit-miyvc2w4')
START = 1791624885599758861
END = START + 60_000_000_000
WARMUP = 10_000_000_000
manifest = json.loads((SNAPSHOT/'manifest.json').read_text())
config_record = next(c for c in manifest['config_evidence'] if c['name']=='field-btf.json')
assert hashlib.sha256(config_record['content_utf8'].encode()).hexdigest() == config_record['sha256']
config = json.loads(config_record['content_utf8'])
assert config['external_output_allowed'] is False

def message(value):
    if isinstance(value, dict):
        return SimpleNamespace(**{k:message(v) for k,v in value.items()})
    if isinstance(value,list):
        return [message(v) for v in value]
    # Capture serializes nonfinite readings as strings.
    if value in ('NaN','Infinity','-Infinity') if isinstance(value,str) else False:
        return float(value)
    return value

class Publisher:
    def __init__(self, node, topic):
        self.node,self.topic=node,topic
    def publish(self, msg):
        if START <= self.node.ros_ns < END:
            self.node.published[self.topic] += 1

class ReplayNode:
    state=production.BtfNode.state
    extended=production.BtfNode.extended
    ground_velocity=production.BtfNode.ground_velocity
    sync=production.BtfNode.sync
    sync_ready=production.BtfNode.sync_ready
    sensor=production.BtfNode.sensor
    tof=production.BtfNode.tof
    imu=production.BtfNode.imu
    raw=production.BtfNode.raw

    def __init__(self, scenario):
        self.scenario=scenario
        self.config=config
        self.require_height=True
        self.processor=MeasuredBtf(config)
        self.recorder=None
        self.counts=collections.Counter()
        self.height_counts=collections.Counter()
        self.last=None
        self.last_source_ns=None
        self.clock_readiness=ClockReadiness()
        self.fc_state=None
        self.fc_state_at=self.landed_at=float('-inf')
        self.landed=None
        self.landed_stamp_ns=0
        self.ground_velocity_ok=False
        self.ground_velocity_at=float('-inf')
        self.ground_height=config.get('ground_antenna_height_m')
        self.ros_ns=self.mono_ns=0
        self.published=collections.Counter()
        self.pose_pub=Publisher(self,'xy')
        self.xyz_pub=Publisher(self,'xyz')
        self.decision_pub=Publisher(self,'decision')
        self.decisions=[]
        self.seeded=False
        self.seed_record=None
        self.input_counts=collections.Counter()
    def get_clock(self):
        return SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=self.ros_ns))
    def record(self, name, row):
        if name=='decisions' and START<=self.ros_ns<END:
            self.decisions.append(row)
    def dispatch(self, row):
        self.ros_ns=row['received_ros_ns']
        self.mono_ns=row['received_monotonic_ns']
        production.time=SimpleNamespace(monotonic=lambda:self.mono_ns/1e9,
                                         perf_counter=time.perf_counter)
        if not self.seeded and self.ros_ns>=START:
            self.seeded=True
            if self.scenario in ('previous_x1m_prior','previous_x1m_then_models_reset'):
                g=self.processor.raw_guard
                g.prior_xy=(4.117692640136927,2.330914612751391)
                g.prior_stamp=START-1_000_000_000
                g.last_seen=START-1_000_000_000
                g.ever_accepted=True
                g.stable_since=None
                if self.scenario.endswith('models_reset'):
                    self.processor.reset_models()
            self.seed_record=dict(self.processor.raw_guard.__dict__)
            self.seed_record['config']=vars(self.processor.raw_guard.config)
        handlers={'/uwb/received':self.raw, config['tof_topic']:self.tof,
                  config['imu_topic']:self.imu,'/mavros/timesync_status':self.sync,
                  '/mavros/state':self.state,'/mavros/extended_state':self.extended,
                  '/mavros/local_position/odom':self.ground_velocity}
        handler=handlers.get(row['topic'])
        if handler:
            self.input_counts[row['topic']]+=1
            handler(message(row['data']))

def stats(v):
    return dict(n=len(v),mean=statistics.mean(v),min=min(v),max=max(v)) if v else None

def report(node):
    ds=node.decisions
    heights=[d['height_m'] for d in ds if d.get('height_m') is not None]
    fits=[d['models']['B']['fit']['rms_m'] for d in ds
          if (d.get('models',{}).get('B',{}).get('fit') or {}).get('rms_m') is not None]
    candidates=[d.get('models',{}).get('B_TF',{}) for d in ds]
    score=[d['best_score_m'] for d in candidates if d.get('best_score_m') is not None]
    reasons=collections.Counter(d['reason'] for d in ds)
    physical=[[] for _ in range(4)]
    physical_rms=[]
    physical_measured=[]
    height_sources=collections.Counter()
    approved_height_sources=collections.Counter()
    b4_hypotheses=[]
    sample_age=[]
    for d in ds:
        height_sources[d.get('height_source','not_reached')]+=1
        if d.get('ok') is not True or d.get('height_m') is None:
            continue
        approved_height_sources[d.get('height_source')]+=1
        xy=d['xy_m'];h=d['height_m']
        residuals=[d['cal_slant_m'][i]-math.dist((xy[0],xy[1],h),a)
                   for i,a in enumerate(config['anchors_xyz_m'])]
        for i,value in enumerate(residuals):physical[i].append(value)
        physical_rms.append(math.sqrt(statistics.mean(r*r for r in residuals)))
        if d.get('height_source')=='measured_tof_imu':
            physical_measured.append(physical_rms[-1])
        for selection in d.get('height_selection',[]):
            if selection.get('tof_age_s') is not None:sample_age.append(selection['tof_age_s'])
    canonical=[]
    for d in ds:
        canonical.append({k:d.get(k) for k in ('seq','reason','ok','xy_m','height_m','height_ready','height_source','published','pose_blocked_reason')})
    g=dict(node.processor.raw_guard.__dict__);g['config']=vars(node.processor.raw_guard.config)
    examples={}
    for d in ds:
        if d['reason'] not in examples:
            examples[d['reason']]=production.safe_json(d)
    return dict(scenario=node.scenario,decisions=len(ds),reasons=dict(reasons),
        published=dict(node.published),input_counts=dict(node.input_counts),
        height_m=stats(heights),B4_fit_rms_m=stats(fits),best_B3_physical_rms_m=stats(score),
        approved_pose_physical_rms_m=stats(physical_rms),
        approved_measured_height_physical_rms_m=stats(physical_measured),
        approved_measured_height_physical_rms_ge_6cm=sum(v>=.06 for v in physical_measured),
        approved_pose_range_residual_by_anchor_m=[stats(v) for v in physical],
        height_sources=dict(height_sources),approved_height_sources=dict(approved_height_sources),
        selected_tof_age_s=stats(sample_age),
        height_ready=sum(d.get('height_ready') is True for d in ds),
        height_selection_reasons=dict(collections.Counter(s['reason'] for d in ds for s in d.get('height_selection',[]))),
        sources=dict(collections.Counter(c.get('source','not_reached') for c in candidates)),
        initial_raw_guard=node.seed_record,final_raw_guard=g,
        decision_digest=hashlib.sha256(json.dumps(canonical,sort_keys=True,allow_nan=False).encode()).hexdigest(),
        examples=examples)

nodes=[ReplayNode(s) for s in ('fresh_session','previous_x1m_prior','previous_x1m_then_models_reset')]
live=collections.Counter()
rows=[]
with gzip.open(SNAPSHOT/'events.jsonl.gz','rt',encoding='utf-8') as f:
    for line in f:
        row=json.loads(line)
        if START-WARMUP<=row['received_ros_ns']<END:
            rows.append(row)
            if row['topic']=='/uwb/btf_decision' and row['received_ros_ns']>=START:
                live[json.loads(row['data']['data'])['reason']]+=1
reports=[]
for i,node in enumerate(nodes):
    for row in rows:
        node.dispatch(row)
    print(json.dumps({'progress':node.scenario,'reasons':dict(collections.Counter(d['reason'] for d in node.decisions))}),flush=True)
    reports.append(report(node))
    (SNAPSHOT/(node.scenario+'.json')).write_text(json.dumps(production.safe_json(reports[-1]),indent=2,allow_nan=False),encoding='utf-8')
    with gzip.open(SNAPSHOT/(node.scenario+'.decisions.jsonl.gz'),'wt',encoding='utf-8') as out:
        for decision in node.decisions:
            out.write(json.dumps(production.safe_json(decision),allow_nan=False)+'\n')
result=dict(schema=1,scope='OFFLINE_PRODUCTION_CALLBACK_REPLAY_NO_ROS_INIT_NO_FC_IO',
    source= json.loads((SNAPSHOT/'provenance.json').read_text()),config_sha256=config_record['sha256'],
    window_ns=[str(START),str(END)],warmup_s=10,reference_xy_known=False,
    live_reasons=dict(live),scenarios=reports,
    limitations=['Recorder callback order/timestamps approximate BtfNode delivery; exact live callback schedule was not recorded.',
                 'Previous-X prior is a controlled history intervention using measured previous station mean; it is not recovered live process memory.',
                 'Pose publishing is simulated into counters; no ROS node, socket, serial or flight command is created.'])
(SNAPSHOT/'replay_report.json').write_text(json.dumps(production.safe_json(result),indent=2,allow_nan=False),encoding='utf-8')
print(json.dumps({'report':str(SNAPSHOT/'replay_report.json'),
                  'scenarios':[{k:s[k] for k in ('scenario','decisions','reasons','published','height_m','B4_fit_rms_m','best_B3_physical_rms_m','approved_pose_physical_rms_m','height_ready','sources')} for s in result['scenarios']]}),flush=True)
