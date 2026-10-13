"""Start one robot with explicit LAN identity and a durable log."""
import argparse,json,os,pathlib,subprocess
p=argparse.ArgumentParser();p.add_argument('robot',choices=['odin_a','odin_b']);p.add_argument('--ip',required=True);p.add_argument('--master',default='192.168.5.21');p.add_argument('--serial',required=True);p.add_argument('--image',default='kimera-odin1:wireless');p.add_argument('--replay',action='store_true');p.add_argument('--backend',choices=['true','false'],default='true');a=p.parse_args()
root=pathlib.Path(__file__).resolve().parents[1];path=root/'artifacts/wireless'/a.robot;path.mkdir(parents=True,exist_ok=True)
env=dict(os.environ,ROS_IP=a.ip,ROS_MASTER_URI=f'http://{a.master}:11311',ODIN_TARGET_USB_SERIAL=a.serial,ODIN_WIRELESS_IMAGE=a.image)
with (path/'runtime_launch.log').open('w') as out:
 proc=subprocess.Popen(['bash',str(root/'docker/wireless_odin1.bash'),'robot',a.robot,'0' if a.robot=='odin_a' else '1','backend:='+a.backend]+(['driver:=false','adapter:=true'] if a.replay else []),stdout=out,stderr=subprocess.STDOUT,env=env,start_new_session=True)
(path/'runtime_process.json').write_text(json.dumps({'pid':proc.pid,'robot':a.robot,'ROS_IP':a.ip,'ROS_MASTER_URI':env['ROS_MASTER_URI'],'usb_serial':a.serial,'image':a.image},indent=2)+'\n')
print('Started',a.robot,'launcher PID',proc.pid,'log',path/'runtime_launch.log')
