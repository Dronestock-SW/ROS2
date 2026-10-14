#!/usr/bin/env python3
"""Read-only deployment gate. Does not open serial ports or issue FC commands."""
import argparse
from collections import Counter
import json
from pathlib import Path
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=8.)
    p.add_argument('--require-backend',choices=['python','cpp'])
    p.add_argument('--require-idle',action='store_true')
    a=p.parse_args()
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from mavros_msgs.msg import State,ExtendedState
    from std_msgs.msg import String
    rclpy.init();n=rclpy.create_node('native_btf_deployment_readback')
    values={};received={};counts=Counter()
    def state(m):
        values['fc']=dict(connected=m.connected,armed=m.armed,mode=m.mode)
        received['fc']=time.monotonic()
    def landed(m):values['landed']=m.landed_state;received['landed']=time.monotonic()
    def status(m):
        try:values['btf']=json.loads(m.data);received['btf']=time.monotonic()
        except ValueError:counts['malformed_status']+=1
    def raw(m):
        try:counts[json.loads(m.data)['message']['type']]+=1
        except (KeyError,ValueError):counts['malformed_raw']+=1
    subscriptions=[n.create_subscription(cls,topic,cb,qos_profile_sensor_data) for cls,topic,cb in (
        (State,'/mavros/state',state),(ExtendedState,'/mavros/extended_state',landed),
        (String,'/uwb/btf_status',status),(String,'/uwb/received',raw))]
    end=time.monotonic()+a.seconds
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
    ages={k:time.monotonic()-v for k,v in received.items()}
    pubs={t:[dict(node=x.node_name,namespace=x.node_namespace) for x in n.get_publishers_info_by_topic(t)]
          for t in ('/uwb/received','/uwb/btf_pose','/mavros/state','/mavros/setpoint_raw/local')}
    fc=values.get('fc',{})
    ready=(fc.get('connected') is True and fc.get('armed') is False and values.get('landed')==1
           and ages.get('fc',99)<1.5 and ages.get('landed',99)<1.5
           and ages.get('btf',99)<2.5 and all(len(pubs[t])==1 for t in ('/uwb/received','/uwb/btf_pose','/mavros/state')))
    if a.require_backend:
        ready=ready and values.get('btf',{}).get('numerical_backend',{'backend':'python'}).get('backend')==a.require_backend
    if a.require_idle:ready=ready and not pubs['/mavros/setpoint_raw/local']
    result=dict(deployment_ground_gate=ready,physical_flight_validated=False,values=values,
                receive_age_s=ages,raw_counts=dict(counts),publishers=pubs)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False))
    n.destroy_node();rclpy.shutdown()
    return 0 if ready else 1

if __name__=='__main__':raise SystemExit(main())
