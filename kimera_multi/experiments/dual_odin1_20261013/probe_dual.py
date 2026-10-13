"""Measure both physical devices or both adapted streams on a wall clock."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image, Imu, PointCloud2, CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import String

parser = argparse.ArgumentParser()
parser.add_argument('--duration',type=float,default=35)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--adapted',action='store_true')
args = parser.parse_args()
args.output.mkdir(parents=True,exist_ok=False)
rclpy.init()
node = Node('dual_odin_observer')
samples,details,imu_values,poses,diagnostics,subscriptions = {},{},{},{},{},[]
streams = {'image/compressed':CompressedImage,'cloud_raw':PointCloud2,'imu':Imu,
           'odometry':Odometry,'odometry_highfreq':Odometry}
if args.adapted:
    streams = {'rgb/image_raw':Image,'depth/image_raw':Image,'rgb/camera_info':CameraInfo,
               'imu':Imu,'reference/odometry':Odometry,'diagnostics':String}
start=time.monotonic()
def callback(msg,topic):
    now=time.monotonic()-start
    if isinstance(msg,String):
        diagnostics[topic]=json.loads(msg.data)
        return
    stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
    samples.setdefault(topic,[]).append((now,stamp,time.time_ns()))
    if topic not in details:
        d=details[topic]={'frame_id':msg.header.frame_id}
        if isinstance(msg,(Image,CameraInfo,PointCloud2)):
            d.update(width=msg.width,height=msg.height)
        if isinstance(msg,Image):
            d['encoding']=msg.encoding
        if isinstance(msg,PointCloud2):
            d.update(fields=[f.name for f in msg.fields],point_step=msg.point_step)
        if isinstance(msg,Odometry):
            d['child_frame_id']=msg.child_frame_id
        if isinstance(msg,CompressedImage):
            payload=bytes(msg.data)
            d.update(format=msg.format,first_payload_sha256=hashlib.sha256(payload).hexdigest())
            (args.output/(topic.split('/')[1]+'.jpg')).write_bytes(payload)
    if isinstance(msg,Imu):
        a,g=msg.linear_acceleration,msg.angular_velocity
        imu_values.setdefault(topic,[]).append([a.x,a.y,a.z,g.x,g.y,g.z])
    if isinstance(msg,Odometry):
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        poses.setdefault(topic,[]).append([p.x,p.y,p.z,q.x,q.y,q.z,q.w])
for slot in ['odin_a','odin_b']:
    for suffix,typ in streams.items():
        topic='/'+slot+('/kimera/' if args.adapted else '/odin1/')+suffix
        qos=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=4000 if suffix=='imu' or suffix=='odometry_highfreq' else 100,
                       reliability=ReliabilityPolicy.RELIABLE)
        subscriptions.append(node.create_subscription(typ,topic,lambda m,t=topic:callback(m,t),qos))
try:
    while time.monotonic()-start<args.duration:
        rclpy.spin_once(node,timeout_sec=.01)
finally:
    result={'duration_wall_s':time.monotonic()-start,'adapted':args.adapted,'topics':{},'diagnostics':diagnostics,
            'expected_topics':['/'+s+('/kimera/' if args.adapted else '/odin1/')+k for s in ['odin_a','odin_b'] for k in streams if k!='diagnostics']}
    for topic,rows in samples.items():
        a=np.asarray(rows,dtype=np.longdouble); dt=np.diff(a[:,1])*1e-6
        d=dict(count=len(rows),sensor_hz=float((len(rows)-1)*1e9/(a[-1,1]-a[0,1])) if len(rows)>1 and a[-1,1]>a[0,1] else None,
               arrival_hz=float((len(rows)-1)/(a[-1,0]-a[0,0])) if len(rows)>1 else None,
               non_increasing=int((dt<=0).sum()),max_gap_ms=float(dt.max()) if len(dt) else None,
               gaps_over_20ms=int((dt>20).sum()),details=details[topic],
               first_stamp_ns=int(rows[0][1]),last_stamp_ns=int(rows[-1][1]),
               reception_minus_header_s_median=float(np.median((a[:,2]-a[:,1])*1e-9)))
        if topic in imu_values:
            values=np.asarray(imu_values[topic]); d['finite']=bool(np.isfinite(values).all())
            d['acceleration_norm_median']=float(np.median(np.linalg.norm(values[:,:3],axis=1)))
            d['gyro_norm_p95']=float(np.percentile(np.linalg.norm(values[:,3:],axis=1),95))
        if topic in poses:
            values=np.asarray(poses[topic]); d['finite']=bool(np.isfinite(values).all())
            d['max_position_from_first_m']=float(np.max(np.linalg.norm(values[:,:3]-values[0,:3],axis=1)))
        result['topics'][topic]=d
        np.save(args.output/(topic.strip('/').replace('/','_')+'_times.npy'),np.asarray(rows,dtype=np.longdouble))
    result['missing_topics']=sorted(set(result['expected_topics'])-set(samples))
    (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2),flush=True)
    node.destroy_node(); rclpy.shutdown()
