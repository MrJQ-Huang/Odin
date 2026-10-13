"""Read every recorded raw message and measure per-device sensor continuity."""
import argparse
import json
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores,get_typestore

parser=argparse.ArgumentParser()
parser.add_argument('bag',type=Path)
parser.add_argument('output',type=Path)
args=parser.parse_args()
times,positions,rotations={}, {}, {}
frames={}
with AnyReader([args.bag],default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
    for connection,arrival,raw in reader.messages():
        msg=reader.deserialize(raw,connection.msgtype)
        if not hasattr(msg,'header'):
            continue
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        times.setdefault(connection.topic,[]).append((stamp,arrival))
        frames[connection.topic]=msg.header.frame_id
        if connection.topic.endswith('/odometry'):
            p,q=msg.pose.pose.position,msg.pose.pose.orientation
            positions.setdefault(connection.topic,[]).append((p.x,p.y,p.z))
            rotations.setdefault(connection.topic,[]).append((q.x,q.y,q.z,q.w))
result={'bag':str(args.bag.resolve()),'topics':{}}
for topic,values in times.items():
    a=np.asarray(values,dtype=np.int64);dt=np.diff(a[:,0])*1e-6
    d=dict(count=len(a),sensor_hz=float((len(a)-1)*1e9/(a[-1,0]-a[0,0])),
           span_s=float((a[-1,0]-a[0,0])*1e-9),first_stamp_ns=int(a[0,0]),last_stamp_ns=int(a[-1,0]),
           max_gap_ms=float(dt.max()),median_gap_ms=float(np.median(dt)),non_increasing=int((dt<=0).sum()),
           gaps_over_20ms=int((dt>20).sum()),gaps_over_3_median=int((dt>3*np.median(dt[dt>0])).sum()),
           frame_id=frames[topic],reception_minus_header_s_median=float(np.median((a[:,1]-a[:,0])*1e-9)))
    if topic in positions:
        xyz=np.asarray(positions[topic]);q=np.asarray(rotations[topic]);q/=np.linalg.norm(q,axis=1,keepdims=True)
        distance=np.linalg.norm(xyz-xyz[0],axis=1)
        angle=np.degrees(2*np.arccos(np.clip(np.abs(q@q[0]),0,1)))
        moving=np.flatnonzero((distance>.05)|(angle>5))
        d.update(finite=bool(np.isfinite(xyz).all() and np.isfinite(q).all()),
                 max_position_from_first_m=float(distance.max()),
                 max_rotation_from_first_deg=float(angle.max()),
                 motion_onset_stamp_ns=int(a[moving[0],0]) if len(moving) else None)
    result['topics'][topic]=d
result['image_cloud_offset_ms']={}
for slot in ['odin_a','odin_b']:
    image=np.asarray(times['/'+slot+'/odin1/image/compressed'])[:,0]
    cloud=np.asarray(times['/'+slot+'/odin1/cloud_raw'])[:,0]
    index=np.searchsorted(cloud,image);before=np.clip(index-1,0,len(cloud)-1);after=np.clip(index,0,len(cloud)-1)
    nearest=np.where(np.abs(cloud[before]-image)<np.abs(cloud[after]-image),cloud[before]-image,cloud[after]-image)*1e-6
    result['image_cloud_offset_ms'][slot]=dict(min=float(nearest.min()),median=float(np.median(nearest)),max=float(nearest.max()))
args.output.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
