"""Replay only original sensor inputs, never recorded algorithm outputs."""
import argparse,json,pathlib,subprocess,time,shlex
p=argparse.ArgumentParser();p.add_argument('--robot',required=True);p.add_argument('--run',required=True);p.add_argument('--seconds',type=int,default=180);a=p.parse_args()
root=pathlib.Path(__file__).resolve().parents[1];out=root/'artifacts/wireless'/a.run/a.robot
bag='/workspace/'+str(out.relative_to(root))+'/data.bag'
topics=['/'+a.robot+'/odin1/'+s for s in ('image/compressed','cloud_raw','imu','odometry_highfreq')]
cmd=['docker','exec','odin-wireless-'+a.robot,'/opt/odin1_entrypoint.bash','rosbag','play','--delay=2','--rate=1','--duration='+str(a.seconds),bag,'--topics']+topics
if subprocess.run(['docker','info'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode:cmd=['sg','docker','-c',shlex.join(cmd)]
with (out/'raw_replay.log').open('w') as f:
 proc=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
(out/'raw_replay.json').write_text(json.dumps({'launcher_pid':proc.pid,'started_host_epoch_s':time.time(),'input_bag':bag,'topics':topics,'rate':1,'scope':'Original Odin raw messages only; new SDK features and backend outputs are computed during this replay.'},indent=2)+'\n');print(proc.pid)
