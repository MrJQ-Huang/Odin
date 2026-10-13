"""Extract a small real-image/BoW/SDK motion audit from a completed local bag."""
import argparse,json,pathlib,collections
import numpy as np
import cv2
import rosbag
p=argparse.ArgumentParser();p.add_argument('bag');p.add_argument('--robot',required=True);p.add_argument('--output',required=True);a=p.parse_args()
out=pathlib.Path(a.output);out.mkdir(parents=True,exist_ok=True)
bows={};times=[];poses=[];counts=collections.Counter();sample_last=-100
with rosbag.Bag(a.bag) as bag:
 topics=['/'+a.robot+s for s in ['/odin1/image/compressed','/odin1/odometry_highfreq','/kimera_vio_ros/bow_query','/kimera_vio_ros/pose_graph_incremental','/kimera_vio_ros/vlc_frames','/kimera/diagnostics','/kimera_vio_ros/sdk_diagnostics']]
 for topic,m,t in bag.read_messages(topics=topics):
  counts[topic]+=1
  if topic.endswith('/bow_query'):
   for q in m.queries:
    if q.robot_id==int(a.robot=='odin_b'):bows[q.pose_id]={'words':dict(zip(q.bow_vector.word_ids,q.bow_vector.word_values))}
  elif topic.endswith('/odometry_highfreq'):
   pos=m.pose.pose.position;poses.append([pos.x,pos.y,pos.z]);times.append(m.header.stamp.to_sec())
  elif topic.endswith('/image/compressed') and t.to_sec()-sample_last>=15:
   sample_last=t.to_sec();img=cv2.imdecode(np.frombuffer(m.data,np.uint8),cv2.IMREAD_COLOR)
   cv2.imwrite(str(out/('image_'+str(int(t.to_sec()))+'.jpg')),cv2.resize(img,(640,518)))
points=np.array(poses)
result={'robot':a.robot,'counts':dict(counts),'sdk_samples':len(times),'sensor_span_s':times[-1]-times[0] if times else 0,'sdk_path_length_m':float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum()) if len(points)>1 else 0,'sdk_position_range_m':np.ptp(points,axis=0).tolist() if len(points) else [],'bows':bows}
(out/'capture_audit.json').write_text(json.dumps(result,indent=2)+'\n');print({k:v for k,v in result.items() if k not in ('counts','bows')})
