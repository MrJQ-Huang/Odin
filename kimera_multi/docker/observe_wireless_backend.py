"""Capture actual backend traffic and optimization paths over the shared master."""
import argparse,collections,json,math,pathlib,time,threading
import rospy
from nav_msgs.msg import Path
from dpgo_ros.msg import Status
from pose_graph_tools_msgs.msg import LoopClosures,PoseGraph,BowQueries,VLCFrames
from pose_graph_tools_msgs.srv import PoseGraphQuery
p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--seconds',type=float,default=40);a=p.parse_args()
rospy.init_node('odin_wireless_backend_observer',anonymous=True)
counts=collections.Counter();last={};errors=[];lock=threading.Lock();subscribers=[];cross_loops=set()
def cb(name,key,m):
    with lock:
        counts[name+'/'+key]+=1
        if key=='status':last[name+'/status']={'robot_id':m.robot_id,'cluster_id':m.cluster_id,'state':m.state,'iteration_number':m.iteration_number,'instance_number':m.instance_number}
        if key=='optimized_path':
            last[name+'/path_poses']=len(m.poses)
            if any(not all(math.isfinite(v) for v in (p.pose.position.x,p.pose.position.y,p.pose.position.z,p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w)) for p in m.poses):errors.append(name+': nonfinite optimization path')
        if key=='loop_closures':
            for e in m.edges:
                if e.robot_from!=e.robot_to:cross_loops.add((e.robot_from,e.key_from,e.robot_to,e.key_to))
for name in ('odin_a','odin_b'):
    for key,typ,suffix in [('status',Status,'/dpgo_ros_node/status'),('optimized_path',Path,'/dpgo_ros_node/path'),('bow',BowQueries,'/kimera_vio_ros/bow_query'),('vlc_responses',VLCFrames,'/kimera_distributed/vlc_responses'),('loop_closures',LoopClosures,'/kimera_distributed/loop_closures')]:
        subscribers.append(rospy.Subscriber('/'+name+suffix,typ,lambda m,n=name,k=key:cb(n,k,m),queue_size=1000,tcp_nodelay=True))
time.sleep(a.seconds)
graphs={}
for rid,name in enumerate(('odin_a','odin_b')):
    srv='/'+name+'/distributed_loop_closure/request_pose_graph'
    rospy.wait_for_service(srv,timeout=10)
    graph=rospy.ServiceProxy(srv,PoseGraphQuery)(rid).pose_graph
    graphs[name]={'nodes':len(graph.nodes),'edges':len(graph.edges),'cross_robot_edges':sum(e.robot_from!=e.robot_to for e in graph.edges)}
result={'wall_duration_s':a.seconds,'counts':dict(counts),'last':last,'distributed_graphs':graphs,'observed_unique_cross_robot_loops':len(cross_loops),'errors':errors,'status':'pass' if all(counts[n+'/status']>0 and counts[n+'/bow']>0 and graphs[n]['nodes']>=2 and counts[n+'/optimized_path']>0 for n in ('odin_a','odin_b')) and not errors else 'fail','scope':'Live SDK frontends, real wireless ROS traffic and backend operation. Static capture does not establish fusion accuracy or motion performance.'}
pathlib.Path(a.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result));raise SystemExit(0 if result['status']=='pass' else 1)
