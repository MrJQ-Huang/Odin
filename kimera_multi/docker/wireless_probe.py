"""Bidirectional ROS TCP transport probe; not an algorithm accuracy test."""
import argparse
import json
import pathlib
import threading
import time
import statistics
import rospy
from std_msgs.msg import String
p=argparse.ArgumentParser(); p.add_argument('mode',choices=['echo','test']);p.add_argument('--output');args=p.parse_args()
rospy.init_node('odin_wireless_'+args.mode,anonymous=True)
if args.mode=='echo':
    pub=rospy.Publisher('/odin_wireless_probe/reply',String,queue_size=20)
    sub=rospy.Subscriber('/odin_wireless_probe/request',String,lambda m:pub.publish(m),queue_size=20,buff_size=4*1024**2,tcp_nodelay=True)
    rospy.spin()
else:
    received={};lock=threading.Lock();sent={};payload='x'*65536
    def cb(m):
        d=json.loads(m.data)
        if d['payload']!=payload: raise ValueError('Transport corruption')
        with lock: received[d['id']]=(time.monotonic()-sent[d['id']])*1000
    sub=rospy.Subscriber('/odin_wireless_probe/reply',String,cb,queue_size=20,buff_size=4*1024**2,tcp_nodelay=True)
    pub=rospy.Publisher('/odin_wireless_probe/request',String,queue_size=20)
    deadline=time.monotonic()+15
    while pub.get_num_connections()==0 and time.monotonic()<deadline:time.sleep(.1)
    if pub.get_num_connections()==0: raise RuntimeError('No remote echo subscriber')
    for i in range(100):
        sent[i]=time.monotonic();pub.publish(String(data=json.dumps({'id':i,'payload':payload})));time.sleep(.1)
    deadline=time.monotonic()+10
    while len(received)<len(sent) and time.monotonic()<deadline: time.sleep(.1)
    values=sorted(received.values())
    d={'kind':'ROS1 TCP bidirectional WiFi echo','payload_bytes':len(payload),'sent':len(sent),'received':len(received),
       'rtt_median_ms':statistics.median(values) if values else None,'rtt_p95_ms':values[int(.95*(len(values)-1))] if values else None,
       'status':'pass' if len(received)==len(sent) else 'fail'}
    if args.output:pathlib.Path(args.output).write_text(json.dumps(d,indent=2)+'\n')
    print(json.dumps(d));raise SystemExit(0 if d['status']=='pass' else 1)
