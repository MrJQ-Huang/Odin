"""Copy per-device calibrated configurations into a writable runtime tree."""
import pathlib
import shutil
import sys
import yaml
root=pathlib.Path(__file__).resolve().parents[1]
name=sys.argv[1]
if name not in ('odin_a','odin_b'): raise ValueError('Unknown robot')
source=root/'artifacts/dual_odin1_20261013'/name
target=root/'artifacts/wireless'/name
target.mkdir(parents=True,exist_ok=True)
for file in ('adapter.yaml','calibration.yaml'): shutil.copy2(source/file,target/file)
shutil.copytree(source/'params',target/'params',dirs_exist_ok=True)
vendor=pathlib.Path(sys.argv[2])
config=yaml.safe_load((vendor/'config/control_command.yaml').read_text())
keys=config['register_keys']
keys.update(use_host_ros_time=0,sendrgb=0,sendrgbcompressed=1,sendrgbundistort=0,
            sendcloudrender=0,senddepth=0,sendreprojection=0,sendoverlay=0,
            send_odom_baselink_tf=0,devstatuslog=0,recorddata=0,save_log=0)
(target/'control_command.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
print(target)
