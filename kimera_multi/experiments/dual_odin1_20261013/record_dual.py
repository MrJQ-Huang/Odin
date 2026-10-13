"""Record only this experiment's raw streams and stop cleanly on a wall clock."""
import argparse
from pathlib import Path
import signal
import subprocess
import time

parser=argparse.ArgumentParser()
parser.add_argument('output')
parser.add_argument('--duration',type=float,default=45)
args=parser.parse_args()
root=Path(__file__).resolve().parent
topics=['/'+s+'/odin1/'+k for s in ['odin_a','odin_b'] for k in
        ['image/compressed','cloud_raw','imu','odometry','odometry_highfreq','wiwc']]
command=['ros2','bag','record','-s','mcap','--max-cache-size','268435456',
         '--qos-profile-overrides-path',str(root/'rosbag2_qos.yaml'),'-o',args.output]+topics
p=subprocess.Popen(command,start_new_session=True)
stop=False
def stopping(signum,frame):
    global stop
    stop=True
signal.signal(signal.SIGINT,stopping)
signal.signal(signal.SIGTERM,stopping)
try:
    deadline=time.monotonic()+args.duration
    while not stop and p.poll() is None and time.monotonic()<deadline:
        time.sleep(.2)
finally:
    if p.poll() is None:
        p.send_signal(signal.SIGINT)
        p.wait(timeout=30)
raise SystemExit(p.returncode)
