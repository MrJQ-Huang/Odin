#!/usr/bin/env python3
"""Independent ROS 2 subscriber for adapter rate, frame and pairing validation."""
import argparse
import json
import time
from collections import Counter
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu, CameraInfo
from std_msgs.msg import String

from odin1_adapter.synchronization import stamp_ns


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--robot-name',default='odin1');p.add_argument('--seconds',type=float,default=40.)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    rclpy.init();node=rclpy.create_node('odin_adapter_observer')
    samples={k:[] for k in ('rgb','depth','imu','info')};details={};diagnostics={}
    def callback(key,msg):
        samples[key].append(stamp_ns(msg))
        details[key]={'frame_id':msg.header.frame_id}
        if isinstance(msg,Image):
            details[key].update(width=msg.width,height=msg.height,encoding=msg.encoding)
    def diag(msg):
        diagnostics.update(json.loads(msg.data))
    prefix='/'+args.robot_name+'/kimera'
    subscriptions=[node.create_subscription(typ,prefix+suffix,lambda msg,key=key:callback(key,msg),
        QoSProfile(depth=4000 if key=='imu' else 50,reliability=ReliabilityPolicy.RELIABLE))
        for key,typ,suffix in [('rgb',Image,'/rgb/image_raw'),('depth',Image,'/depth/image_raw'),
            ('imu',Imu,'/imu'),('info',CameraInfo,'/rgb/camera_info')]]
    subscriptions.append(node.create_subscription(String,prefix+'/diagnostics',diag,10))
    start=time.monotonic()
    while time.monotonic()-start<args.seconds:
        rclpy.spin_once(node,timeout_sec=.1)
    result={'duration_wall_s':time.monotonic()-start,'topics':{},'adapter_diagnostics':diagnostics}
    for key,values in samples.items():
        a=np.asarray(values,np.int64);dt=np.diff(a)
        result['topics'][key]={'count':len(a),'details':details.get(key),
            'sensor_stamp_hz':float((len(a)-1)*1e9/(a[-1]-a[0])) if len(a)>1 else None,
            'max_header_gap_ms':float(dt.max()*1e-6) if len(dt) else None,
            'non_increasing_stamps':int((dt<=0).sum())}
    result['rgb_depth_common_stamps']=len(set(samples['rgb'])&set(samples['depth']))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    node.destroy_node();rclpy.shutdown()
    if not samples['rgb'] or not samples['depth'] or not samples['imu']:
        raise SystemExit('Missing adapter output')


if __name__=='__main__':
    main()
