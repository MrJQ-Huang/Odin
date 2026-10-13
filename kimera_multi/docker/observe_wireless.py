"""Inspect local device/frontend output without transporting raw remote RGB-D."""
import argparse
import collections
import json
import math
import pathlib
import threading
import time
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, PointCloud2, Image, Imu
from std_msgs.msg import String
from pose_graph_tools_msgs.msg import PoseGraph, BowQueries, VLCFrames
p=argparse.ArgumentParser();p.add_argument('--robot',required=True);p.add_argument('--seconds',type=float,default=40);p.add_argument('--output',required=True);args=p.parse_args()
rospy.init_node(args.robot+'_wireless_observer',anonymous=True)
counts=collections.Counter();stamps=collections.defaultdict(list);last={};errors=[];lock=threading.Lock();rawposes={};pending=[];initial_diagnostics={}
def cb(key,m):
    with lock:
        counts[key]+=1
        if hasattr(m,'header') and key != 'bow': stamps[key].append(m.header.stamp.to_nsec())
        if key in ('raw_sdk','sdk'):
            q=m.pose.pose.orientation;t=m.pose.pose.position; values=(t.x,t.y,t.z,q.x,q.y,q.z,q.w)
            if not all(math.isfinite(x) for x in values):errors.append(key+': nonfinite pose')
            ns=m.header.stamp.to_nsec()
            if key=='raw_sdk':rawposes[ns]=values
            else:pending.append((ns,values))
        if key in ('adapter_diagnostics','sdk_diagnostics'):
            last[key]=json.loads(m.data)
            if key not in initial_diagnostics: initial_diagnostics[key]=last[key]
        if key=='graph':last['graph_nodes']=len(m.nodes);last['graph_edges']=len(m.edges)
        if key=='vlc' and m.frames:last['latest_vlc_id']=m.frames[-1].pose_id;last['latest_vlc_features']=len(m.frames[-1].depths)
spec=[('raw_rgb',CompressedImage,'/odin1/image/compressed'),('raw_cloud',PointCloud2,'/odin1/cloud_raw'),('raw_imu',Imu,'/odin1/imu'),('raw_sdk',Odometry,'/odin1/odometry_highfreq'),('rgbd',Image,'/kimera/depth/image_raw'),('sdk',Odometry,'/kimera_vio_ros/odometry'),('graph',PoseGraph,'/kimera_vio_ros/pose_graph'),('bow',BowQueries,'/kimera_vio_ros/bow_query'),('vlc',VLCFrames,'/kimera_vio_ros/vlc_frames'),('adapter_diagnostics',String,'/kimera/diagnostics'),('sdk_diagnostics',String,'/kimera_vio_ros/sdk_diagnostics')]
subs=[rospy.Subscriber('/'+args.robot+suffix,typ,lambda m,k=key:cb(k,m),queue_size=2000 if key in ('sdk','raw_sdk','raw_imu') else 10,buff_size=32*1024**2,tcp_nodelay=True) for key,typ,suffix in spec]
start=time.monotonic();time.sleep(args.seconds)
with lock:
    duration=time.monotonic()-start
    comparisons=[(ns,v,rawposes[ns]) for ns,v in pending if ns in rawposes]
    changes=[max(abs(a-b) for a,b in zip(v,raw)) for _,v,raw in comparisons]
    monotonic={k:all(b>a for a,b in zip(v,v[1:])) for k,v in stamps.items()}
    rates={k:(len(v)-1)*1e9/(v[-1]-v[0]) for k,v in stamps.items() if len(v)>1 and v[-1]>v[0]}
    output={'robot':args.robot,'wall_duration_s':duration,'note':'BoW headers describe communication envelopes and may repeat across destination robot IDs; sensor monotonicity is checked on sensor streams.','counts':dict(counts),'sensor_rates_hz':rates,'strictly_increasing':monotonic,'last':last,'sdk_pose_comparisons':len(comparisons),'sdk_max_pose_component_change':max(changes) if changes else None,'diagnostic_counter_changes':{k:{counter:value-initial_diagnostics[k].get('counters',{}).get(counter,0) for counter,value in data.get('counters',{}).items() if counter!='rgbd_processing_ms_last'} for k,data in last.items() if k in initial_diagnostics},'errors':errors[:20], 'status':'pass' if all(counts[k]>0 for k in ('raw_rgb','raw_cloud','raw_imu','raw_sdk','rgbd','sdk','graph','bow','vlc')) and comparisons and max(changes)==0 and not errors and all(monotonic.values()) else 'fail'}
pathlib.Path(args.output).write_text(json.dumps(output,indent=2)+'\n');print(json.dumps(output));raise SystemExit(0 if output['status']=='pass' else 1)
