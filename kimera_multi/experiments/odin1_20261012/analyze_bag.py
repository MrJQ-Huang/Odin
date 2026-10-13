#!/usr/bin/python3
"""Compare recorded header continuity with the independent live observer."""
import argparse
import json
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from probe import percentiles, stamp

root = Path(__file__).resolve().parent
parser = argparse.ArgumentParser()
parser.add_argument('--bag', type=Path, default=root/'motion_bag')
parser.add_argument('--output', type=Path, default=root)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(args.bag),storage_id='mcap'),
            rosbag2_py.ConverterOptions('',''))
types = {t.name:get_message(t.type) for t in reader.get_all_topics_and_types() if t.name.startswith('/odin1/')}
reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
rows = {key:[] for key in types}
while reader.has_next():
    topic, data, received_ns = reader.read_next()
    msg = deserialize_message(data,types[topic])
    rows[topic].append([received_ns*1e-9,stamp(msg)])
result = {'bag': str(args.bag.resolve()), 'topics':{}}
for key, values in rows.items():
    a = np.asarray(values)
    dt = np.diff(a[:,1])
    result['topics'][key] = dict(count=len(values),
        span_s=float(a[-1,1]-a[0,1]),
        stamp_hz=float((len(values)-1)/(a[-1,1]-a[0,1])),
        stamp_dt_ms=percentiles(dt*1000), non_increasing_stamps=int((dt<=0).sum()),
        gaps_over_3_median=int((dt>3*np.median(dt[dt>0])).sum()))
    np.savetxt(args.output/('bag_'+key.rsplit('/',1)[-1]+'_timestamps.csv'),a,delimiter=',',
               header='recorded_wall_stamp_s,header_stamp_s')
for source,target in [('/odin1/image/compressed','/odin1/cloud_raw'),
                      ('/odin1/image/compressed','/odin1/odometry')]:
    a=np.asarray(rows[source])[:,1]; b=np.sort(np.asarray(rows[target])[:,1])
    idx=np.searchsorted(b,a); idx0=np.clip(idx-1,0,len(b)-1); idx1=np.clip(idx,0,len(b)-1)
    nearest=np.where(np.abs(b[idx0]-a)<=np.abs(b[idx1]-a),b[idx0]-a,b[idx1]-a)
    result.setdefault('nearest_stamp_offsets_ms',{})[source+'->'+target]=percentiles(nearest*1000)
(args.output/'bag_summary.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
