"""Sample host and experiment process use without installing dependencies."""
import argparse
import json
import os
from pathlib import Path
import time

parser=argparse.ArgumentParser()
parser.add_argument('--duration',type=float,default=60)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
hz=os.sysconf('SC_CLK_TCK')
previous,rows={},[]
start=time.monotonic()
previous_cpu=None
while time.monotonic()-start<args.duration:
    now=time.monotonic()
    memory={line.split(':')[0]:int(line.split()[1])*1024 for line in Path('/proc/meminfo').read_text().splitlines() if line.split()[1].isdigit()}
    cpu=[int(x) for x in Path('/proc/stat').read_text().splitlines()[0].split()[1:9]]
    total,idle=sum(cpu),cpu[3]+cpu[4]
    sample={'elapsed_s':now-start,'memory_available_bytes':memory['MemAvailable'],
            'swap_used_bytes':memory['SwapTotal']-memory['SwapFree'],'processes':{}}
    if previous_cpu:
        sample['host_busy_percent']=100*(1-(idle-previous_cpu[1])/(total-previous_cpu[0]))
    previous_cpu=(total,idle)
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            command=(path/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
            if not any(key in command for key in ['host_sdk_sample','odin1-adapter','kimera_vio_ros_node','odin_sdk_frontend','ros2 bag record']):
                continue
            stats=(path/'stat').read_text().rsplit(')',1)[1].split()
            ticks=int(stats[11])+int(stats[12]); rss=int(stats[21])*os.sysconf('SC_PAGE_SIZE')
            data={'command':command,'rss_bytes':rss}
            if path.name in previous:
                old_time,old_ticks=previous[path.name]
                data['cpu_percent_one_core']=100*(ticks-old_ticks)/hz/(now-old_time)
            previous[path.name]=(now,ticks)
            sample['processes'][path.name]=data
        except (OSError,ValueError,IndexError):
            pass
    rows.append(sample)
    args.output.write_text(json.dumps(rows,indent=2)+'\n')
    time.sleep(1)
