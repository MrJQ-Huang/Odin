"""Record raw local Odin inputs and algorithm outputs, with a bounded duration."""
import argparse,json,pathlib,subprocess,time
p=argparse.ArgumentParser();p.add_argument('--robot',choices=['odin_a','odin_b'],required=True);p.add_argument('--run',required=True);p.add_argument('--seconds',type=int,default=180);a=p.parse_args()
root=pathlib.Path(__file__).resolve().parents[1];out=root/'artifacts/wireless'/a.run/a.robot;out.mkdir(parents=True,exist_ok=True)
name=a.robot; topics=['/'+name+s for s in ['/odin1/image/compressed','/odin1/cloud_raw','/odin1/imu','/odin1/odometry','/odin1/odometry_highfreq','/kimera/diagnostics','/kimera_vio_ros/odometry','/kimera_vio_ros/keyframe_odometry','/kimera_vio_ros/pose_graph_incremental','/kimera_vio_ros/pose_graph','/kimera_vio_ros/bow_query','/kimera_vio_ros/vlc_frames','/kimera_vio_ros/sdk_diagnostics','/kimera_distributed/loop_closures','/kimera_distributed/pose_graph_incremental','/kimera_distributed/vlc_requests','/kimera_distributed/vlc_responses','/kimera_distributed/optimized_nodes','/kimera_distributed/optimized_path','/dpgo_ros_node/status','/dpgo_ros_node/path','/dpgo_ros_node/optimized_pose_graph','/dpgo_ros_node/public_poses','/dpgo_ros_node/public_measurements']]
container='odin-wireless-'+name
bag='/workspace/'+str(out.relative_to(root))+'/data.bag'
cmd=['docker','exec',container,'/opt/odin1_entrypoint.bash','rosbag','record','--lz4','--buffsize=512','--duration='+str(a.seconds),'-O',bag]+topics
if subprocess.run(['docker','info'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode:
 import shlex
 cmd=['sg','docker','-c',shlex.join(cmd)]
with (out/'recording.log').open('w') as f:
 proc=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
(out/'recording.json').write_text(json.dumps({'robot':name,'run':a.run,'duration_limit_s':a.seconds,'started_host_epoch_s':time.time(),'launcher_pid':proc.pid,'topics':topics,'bag':bag},indent=2)+'\n')
print('Recording',name,'PID',proc.pid,'to',out/'data.bag')
