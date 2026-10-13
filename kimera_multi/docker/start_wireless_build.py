"""Start a logged extension build, without tying it to an SSH session."""
import argparse,json,os,pathlib,subprocess,shlex
p=argparse.ArgumentParser();p.add_argument('--base',default='kimera-odin1:noetic');p.add_argument('--image',default='kimera-odin1:wireless');p.add_argument('--jobs',default='2');p.add_argument('--label',default='local');a=p.parse_args()
root=pathlib.Path(__file__).resolve().parents[1]
log=root/'artifacts/docker'/f'{a.label}_wireless_build.log'
cmd=['docker','build','--network=host','--build-arg','BASE_IMAGE='+a.base,'--build-arg','BUILD_JOBS='+a.jobs,'-f',str(root/'artifacts/docker/wireless-context/docker/Dockerfile.odin1-wireless'),'-t',a.image,str(root/'artifacts/docker/wireless-context')]
if subprocess.run(['docker','info'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode:cmd=['sg','docker','-c',shlex.join(cmd)]
with log.open('w') as out:
 proc=subprocess.Popen(cmd,stdout=out,stderr=subprocess.STDOUT,env=dict(os.environ,DOCKER_BUILDKIT='0'),start_new_session=True)
(root/'artifacts/docker'/f'{a.label}_wireless_process.json').write_text(json.dumps({'pid':proc.pid,'log':str(log),'image':a.image}))
print(proc.pid)
