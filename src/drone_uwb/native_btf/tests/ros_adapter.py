"""Localhost domain 99 only: RAW -> real BtfNode/C++ -> stamped pose/decision."""
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time

assert os.environ.get('ROS_DOMAIN_ID')=='99' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from drone_uwb.integration.ros.btf_node import BtfNode

root=Path(__file__).resolve().parents[2]
config=json.loads((root/'config/runtime/uwb_btf_real.json').read_text())
config.update(numerical_backend='cpp',native_btf_library=str(Path(sys.argv[1]).resolve()),tdma_mode='auto')
with tempfile.TemporaryDirectory(prefix='btf-ros-99-') as tmp:
    cfg=Path(tmp)/'config.json';cfg.write_text(json.dumps(config))
    rclpy.init(args=['--ros-args','-p','config_file:='+str(cfg),'-p','require_height_for_pose:=true'])
    node=BtfNode();peer=rclpy.create_node('synthetic_ranges_no_fc');ex=SingleThreadedExecutor()
    ex.add_node(node);ex.add_node(peer)
    decisions=[];poses=[]
    subs=[peer.create_subscription(String,'/uwb/btf_decision',lambda m:decisions.append(json.loads(m.data)),qos_profile_sensor_data),
          peer.create_subscription(PoseWithCovarianceStamped,'/uwb/btf_pose',lambda m:poses.append(m),qos_profile_sensor_data)]
    pub=peer.create_publisher(String,'/uwb/received',10)
    def spin(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:ex.spin_once(timeout_sec=.001)
    def send(msg):
        event=dict(message=msg,host_received_monotonic_ns=time.monotonic_ns(),host_received_ros_ns=peer.get_clock().now().nanoseconds)
        pub.publish(String(data=json.dumps(event)))
    status=dict(schema=1,type='uwb_raw_status',tag_id=config['tag_id'],clock_domain='esp32_monotonic_boot_us',
                anchor_order=['A1','A2','A3','A4'],anchor_count=4,uwb_ready=True,temporal_filter_applied=False)
    try:
        spin(.5);base=time.monotonic_ns();send(status);spin(.025)
        for k in range(1,181):
            end=100000+(time.monotonic_ns()-base)//1000
            times=[end-13000+i*4000 for i in range(4)]
            ranges=[math.dist([2.,2.,1.12],a) for a in config['anchors_xyz_m']]
            if 65<=k<80:ranges[2]+=.9
            send(dict(schema=1,type='uwb_raw_cycle',tag_id=config['tag_id'],seq=k,cycle_start_us=end-16000,
                      cycle_end_us=end,valid_mask=15,raw_slant_m=ranges,sample_time_us=times,failure=['ok']*4))
            spin(.025)
            if k%30==0:send(status);spin(.001)
        spin(.15)
        valid=[d for d in decisions if d.get('published')]
        rejected=[d for d in decisions if not d.get('published') and d.get('stamp_ns')]
        stamps={d['stamp_ns'] for d in valid}
        assert len(poses)>30 and len(rejected)>5,(len(poses),len(rejected))
        assert all(m.header.stamp.sec*1000000000+m.header.stamp.nanosec in stamps for m in poses)
        assert all(d['decision_stamp_ns']>=d['stamp_ns'] for d in rejected)
        assert all(d['numerical_backend']['backend']=='cpp' for d in valid)
        assert all(d['xyz_m'] is None for d in valid),'missing ToF invented altitude'
        assert not peer.get_publishers_info_by_topic('/mavros/setpoint_raw/local')
        print(json.dumps(dict(domain=99,poses=len(poses),rejections=len(rejected),source_stamps_preserved=True,
                              native_library=node.processor.native.identity,physical_flight=False)))
    finally:
        ex.shutdown();node.destroy_node();peer.destroy_node();rclpy.shutdown()
