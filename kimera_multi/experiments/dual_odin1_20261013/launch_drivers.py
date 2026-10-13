"""Supervise only this experiment's USB-bound driver processes."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

root = Path(__file__).resolve().parent
stop = False
def stopping(signum, frame):
    global stop
    stop = True
signal.signal(signal.SIGINT, stopping)
signal.signal(signal.SIGTERM, stopping)
processes, logs = [], []
exe = '/home/uros/odin/ros2_ws/install/odin_ros_driver/lib/odin_ros_driver/host_sdk_sample'
filter_lib = '/home/uros/odin/ros2_ws/install/odin_ros_driver/lib/libusb_odin_filter.so'
try:
    for device in json.loads((root/'devices.json').read_text()):
        slot = device['slot']
        env = os.environ.copy()
        env.update(ODIN_USB_BUS=str(device['usb_bus']), ODIN_USB_ADDR=str(device['usb_addr']),
                   ODIN_CALIB_DIR=device['calib_dir'], LD_PRELOAD=filter_lib)
        log = (root/slot/'driver.log').open('w')
        logs.append(log)
        command = [exe, '--ros-args', '-r', '__ns:=/'+slot, '-r', '__node:=host_sdk_sample',
                   '-p', 'config_file:='+device['config'], '-p', 'frame_prefix:='+slot,
                   '-p', 'command_file:='+str(root/slot/'commands.txt')]
        p = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                             start_new_session=True, cwd=root/slot)
        processes.append((slot,p))
    (root/'driver_pids.json').write_text(json.dumps({'supervisor':os.getpid(),
        'drivers':{slot:p.pid for slot,p in processes}},indent=2))
    end = time.monotonic()+1800
    while not stop and time.monotonic()<end:
        if any(p.poll() is not None for _,p in processes):
            break
        time.sleep(.5)
finally:
    for slot,p in reversed(processes):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:
                p.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGTERM)
                p.wait(timeout=8)
    for log in logs:
        log.close()
    (root/'driver_exit_codes.json').write_text(json.dumps({slot:p.returncode for slot,p in processes},indent=2))
