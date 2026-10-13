"""Query real robot frontend services across the shared LAN master."""
import json,pathlib,sys,time
import rospy
from pose_graph_tools_msgs.srv import PoseGraphQuery,VLCFrameQuery
from pose_graph_tools_msgs.msg import VLCFrames
rospy.init_node('odin_wireless_service_validator',anonymous=True)
output=[]
for name,rid in [('odin_a',0),('odin_b',1)]:
    prefix='/'+name+'/kimera_vio_ros'
    rospy.wait_for_service(prefix+'/pose_graph_query',timeout=20)
    try:
        sample=rospy.wait_for_message(prefix+'/vlc_frames',VLCFrames,timeout=2)
        chosen=[f.pose_id for f in sample.frames if f.robot_id==rid]
    except rospy.ROSException:
        chosen=[]
    start=time.monotonic()
    graph=rospy.ServiceProxy(prefix+'/pose_graph_query',PoseGraphQuery)(rid).pose_graph
    ids=[n.key for n in graph.nodes]
    if not chosen: chosen=ids[-1:]
    frames=rospy.ServiceProxy(prefix+'/vlc_frame_query',VLCFrameQuery)(rid,chosen).frames
    output.append({'robot':name,'robot_id':rid,'graph_nodes':len(graph.nodes),'graph_edges':len(graph.edges),'queried_pose_ids':chosen,'queried_ids_in_graph':all(i in ids for i in chosen),'vlc_frames':len(frames),'feature_rows':sum(len(f.depths) for f in frames),'query_wall_ms':(time.monotonic()-start)*1000,'robot_ids_correct':all(n.robot_id==rid for n in graph.nodes) and all(f.robot_id==rid for f in frames)})
result={'kind':'actual SDK graph / visual geometry service queries from local PC','robots':output,'status':'pass' if all(x['graph_nodes']>=2 and x['graph_edges']>=1 and x['vlc_frames']>0 and x['feature_rows']>0 and x['robot_ids_correct'] and x['queried_ids_in_graph'] for x in output) else 'fail'}
pathlib.Path(sys.argv[1]).write_text(json.dumps(result,indent=2)+'\n'); print(json.dumps(result));raise SystemExit(0 if result['status']=='pass' else 1)
