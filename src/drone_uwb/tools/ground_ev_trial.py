#!/usr/bin/env python3
"""Explicit, bounded, static Tag B EV transport trial. No flight or parameter commands.

Candidate frame/timing values remain unconfirmed. Production bridge and mission
outputs must stay disabled. Fresh ground, RC kill, clocks and parameter readback
are required throughout; this tool never changes a production approval flag.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import time
import signal
from collections import deque


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=45.)
    parser.add_argument('--continuous-ground',action='store_true',
        help='Keep this boot\'s static DISARM/kill-ON observation stream running; never enables flight')
    args=parser.parse_args()
    if not 5 <= args.seconds <= 60 or os.environ.get('ROS_DOMAIN_ID')!='2':
        parser.error('Tag B domain 2 and bounded 5..60 seconds required')
    cfg=json.loads(args.candidate.read_text(encoding='utf-8'))
    if args.continuous_ground and cfg.get('boot_id')!=Path('/proc/sys/kernel/random/boot_id').read_text().strip():
        parser.error('Continuous ground mode requires this boot\'s captured reference')
    if cfg.get('purpose')!='DISARMED_UWB_EV_TRANSPORT_TRIAL' or cfg.get('flight_authorized') is not False:
        parser.error('Explicit ground-only candidate required')
    for name in ('enu_yaw_deg','enu_offset_x_m','enu_offset_y_m'):
        if type(cfg.get(name)) not in (int,float) or not math.isfinite(cfg[name]):
            parser.error('Finite fixed candidate transform required')
    if cfg.get('map_y_axis_sign') not in (-1,1) or not cfg.get('evidence_reference'):
        parser.error('Explicit axis convention and candidate evidence required')
    mount=cfg.get('measured_mount_frd_m')
    if (cfg.get('sensor_mount_confirmed') is not True or not isinstance(mount,list) or len(mount)!=3
            or any(type(x) not in (int,float) or not math.isfinite(x) or abs(x)>1 for x in mount)):
        parser.error('Explicit measured FC-to-antenna FRD vector required')
    args.output.mkdir(parents=True,exist_ok=False)

    import rclpy
    from rclpy.qos import qos_profile_sensor_data as qos
    from mavros_msgs.msg import RCIn,ExtendedState
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from drone_uwb.integration.ros import bridge
    from drone_uwb.processing.ground_ev_trial import ground_trial_gate, inflated_xy_covariance, GroundAuthorityLatch
    from drone_mission.writer_lock import WriterLock

    bridge.PARAMETERS=(*bridge.PARAMETERS,'RC_MAP_KILL_SW')
    remaps=['--ros-args','-r','__node:=uwb_ground_ev_trial',
        '-p','input_source:=btf_xy','-p',"tag_id:='6'",
        '-p','ground_only:=true','-p','enabled:=false']
    # Production confirmation parameters retain their false defaults.
    for key in ('enu_yaw_deg','enu_offset_x_m','enu_offset_y_m','map_y_axis_sign'):
        remaps+=['-p',f'{key}:={cfg[key]}']
    for axis,value in zip('xyz',mount):
        remaps+=['-p',f'antenna_body_frd_{axis}_m:={float(value)}']
    rclpy.init(args=remaps)

    class Trial(bridge.UwbPx4Bridge):
        def __init__(self):
            self.authority=GroundAuthorityLatch()
            super().__init__()
            # Do not remap the topic: the production-status subscriber below
            # must still observe the real production bridge.
            self.destroy_publisher(self.status_pub)
            self.status_pub=self.create_publisher(String,'/diagnostic/uwb_ground_ev_status',10)
            self.landed=None; self.kill_channel=0
            self.landed_at=self.rc_at=self.production_at=self.flight_at=self.odom_at=float('-inf')
            self.production_disabled=self.execution_disabled=False
            self.static_ok=False; self.baseline=None; self.rows=deque(maxlen=1000)
            self.create_subscription(ExtendedState,'/mavros/extended_state',self.extended,qos)
            self.create_subscription(RCIn,'/mavros/rc/in',self.rc,qos)
            self.create_subscription(String,'/uwb/bridge_status',self.production,qos)
            self.create_subscription(String,'/flight_state',self.flight,qos)
            self.create_subscription(Odometry,'/mavros/local_position/odom',self.odom,qos)

        def fresh_stamp(self,msg,limit):
            stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            return stamp>0 and 0 <= (self.get_clock().now().nanoseconds-stamp)/1e9 <= limit

        def extended(self,msg):
            fresh=self.fresh_stamp(msg,1.5)
            self.authority.observe(fresh=fresh,landed=msg.landed_state)
            self.landed=msg.landed_state if fresh else None
            self.landed_at=time.monotonic()

        def rc(self,msg):
            if len(msg.channels)>=8 and self.fresh_stamp(msg,.3):
                self.kill_channel=msg.channels[6];self.rc_at=time.monotonic()
                self.authority.observe(fresh=True,kill_channel=self.kill_channel)

        def receive_state(self,msg):
            super().receive_state(msg)
            self.authority.observe(fresh=self.fresh_stamp(msg,1.5),armed=msg.armed,connected=msg.connected)

        def production(self,msg):
            try: row=json.loads(msg.data)
            except ValueError: self.production_disabled=False; return
            self.production_disabled=(row.get('gate')=='disabled' and row.get('published')==0
                and row.get('settings',{}).get('enabled') is False)
            if not self.production_disabled:self.authority.revoked=True
            self.production_at=time.monotonic()

        def flight(self,msg):
            try: row=json.loads(msg.data)
            except ValueError: self.execution_disabled=False; return
            checks={c['code']:c['passed'] for c in row.get('preflight',{}).get('checks',[])}
            self.execution_disabled=(checks.get('execution') is False and row.get('state')=='IDLE'
                and row.get('control_ack') is None and row.get('flight_control_enabled') is False)
            if not self.execution_disabled:self.authority.revoked=True
            self.flight_at=time.monotonic()

        def odom(self,msg):
            p=msg.pose.pose.position;v=msg.twist.twist.linear
            values=(p.x,p.y,p.z,v.x,v.y,v.z)
            self.static_ok=(self.fresh_stamp(msg,.2) and msg.header.frame_id=='map'
                and all(math.isfinite(x) for x in values) and math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)<.2)
            if self.static_ok and self.baseline is None: self.baseline=(p.x,p.y)
            self.static_ok=self.static_ok and self.baseline is not None and math.dist((p.x,p.y),self.baseline)<.3
            self.odom_at=time.monotonic()

        def current_gate(self):
            if self.authority.revoked:return 'ground_authority_revoked'
            now=time.monotonic()
            reason=ground_trial_gate(connected=self.connected,armed=self.armed,state_age_s=now-self.state_time,
                landed=self.landed,landed_age_s=now-self.landed_at,kill_channel=self.kill_channel,rc_age_s=now-self.rc_at,
                bridge_disabled=self.production_disabled,bridge_age_s=now-self.production_at,
                execution_disabled=self.execution_disabled,flight_age_s=now-self.flight_at,
                parameters=self.params,parameter_age_s=now-self.param_time,measured_mount_frd_m=mount)
            if reason!='ready':return reason
            if not self.static_ok or now-self.odom_at>.2:return 'stationary_ground_required'
            if self.sync_count<30 or not 0<=now-self.sync_time<.5:return 'stable_timesync_required'
            if self.count_publishers(self.input_topic)!=1 or self.count_publishers('/mavros/state')!=1:
                return 'unique_source_required'
            names={x.node_name for x in self.get_publishers_info_by_topic('/mavros/vision_pose/pose_cov')}
            if names!={'uwb_px4_bridge','uwb_ground_ev_trial'} or self.count_publishers('/mavros/vision_pose/pose_cov')!=2:
                return 'unexpected_vision_writer'
            return 'ready'

        def receive_pose(self,msg):
            candidate=copy.deepcopy(msg)
            # Explicit conservative test allowance, NOT a measured noise calibration.
            try:
                covariance=inflated_xy_covariance([msg.pose.covariance[i] for i in (0,1,6,7)])
            except ValueError:
                self.last_reason='invalid_source_covariance';self.rejected+=1;return
            for index,value in zip((0,1,6,7),covariance):
                candidate.pose.covariance[index]=value
            before=self.published
            super().receive_pose(candidate)
            if self.published!=before:
                self.rows.append(dict(source_stamp_ns=self.last_stamp,
                    source_xy_m=[msg.pose.pose.position.x,msg.pose.pose.position.y],
                    source_covariance_xy=[msg.pose.covariance[i] for i in (0,1,6,7)]))

    lock=WriterLock(2)
    node=Trial(); started=time.monotonic(); first=None; result='startup_timeout';saved_at=0.
    def report():
        return dict(scope='continuous_disarmed_static_ev' if args.continuous_ground else 'disarmed_static_ev_transport_trial_not_flight',
            candidate=cfg,alignment_confirmed=False,timing_confirmed=False,fusion_confirmed=False,
            flight_authorized=False,physical_flight_commands=False,fc_parameter_writes=False,
            result=result,gate=node.current_gate(),last_reason=node.last_reason,
            published=node.published,rejected=node.rejected,duration_s=time.monotonic()-started,
            last_source_stamp_ns=node.last_stamp,covariance_test_allowance_m2=.25,
            parameters=node.params,source_samples=list(node.rows))
    def save():
        temporary=args.output/'summary.tmp'
        temporary.write_text(json.dumps(report(),indent=2),encoding='utf-8')
        temporary.replace(args.output/'summary.json')
    def stop(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop)
    try:
        while rclpy.ok() and (args.continuous_ground or time.monotonic()-started<args.seconds+20):
            rclpy.spin_once(node,timeout_sec=.02)
            now=time.monotonic()
            if now-saved_at>=2:
                save();saved_at=now
            if node.published and first is None:
                first=now
                result='streaming_ground_only'
                print('GROUND_EV_PUBLISHING',flush=True)
            if first is not None:
                reason=node.current_gate()
                # Short sensor/clock gaps pause publication. Authority changes
                # terminate the session and require a new explicit invocation.
                recoverable={'stable_timesync_required','stationary_ground_required','unique_source_required',
                    'fresh_disarmed_fc_required','fresh_ground_state_required','mapped_kill_switch_on_required',
                    'production_bridge_must_remain_disabled','mission_output_must_remain_disabled','parameters_stale'}
                if reason!='ready' and not (args.continuous_ground and reason in recoverable):
                    result=reason;break
                if not args.continuous_ground and now-first>=args.seconds:result='duration_complete';break
            elif now-started>20:break
    except KeyboardInterrupt:
        result='interrupted'
    finally:
        save();final_report=report()
        node.destroy_node();rclpy.shutdown()
        lock.close()
        print(json.dumps({k:v for k,v in final_report.items() if k!='source_samples'},indent=2),flush=True)
    return 0 if result=='duration_complete' and final_report['published']>0 else 2


if __name__=='__main__':
    raise SystemExit(main())
