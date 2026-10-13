"""Save a small pose timeline while recording, to select an actual motion interval."""
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

root=Path(__file__).resolve().parent
timeline={'started_unix_s':time.time(),'devices':{s:[] for s in ['odin_a','odin_b']}}
rclpy.init();node=Node('dual_odin_motion_timeline');subs=[]
def callback(msg,slot):
    p,q=msg.pose.pose.position,msg.pose.pose.orientation
    row=dict(stamp_s=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
             received_unix_s=time.time(),position=[p.x,p.y,p.z],quaternion_xyzw=[q.x,q.y,q.z,q.w])
    rows=timeline['devices'][slot]
    if rows:
        row['distance_from_first_m']=math.sqrt(sum((a-b)**2 for a,b in zip(row['position'],rows[0]['position'])))
        first=rows[0]['quaternion_xyzw'];dot=sum(a*b for a,b in zip(row['quaternion_xyzw'],first))
        norm=math.sqrt(sum(x*x for x in row['quaternion_xyzw'])*sum(x*x for x in first))
        row['angle_from_first_deg']=math.degrees(2*math.acos(min(1,abs(dot)/norm)))
    else:
        row.update(distance_from_first_m=0,angle_from_first_deg=0)
    rows.append(row)
for slot in timeline['devices']:
    subs.append(node.create_subscription(Odometry,'/'+slot+'/odin1/odometry',lambda m,s=slot:callback(m,s),qos_profile_sensor_data))
end=time.monotonic()+480
last_save=0
try:
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.02)
        if time.monotonic()-last_save>=1:
            last_save=time.monotonic()
            (root/'motion_usb128_timeline.json').write_text(json.dumps(timeline,indent=2)+'\n')
finally:
    (root/'motion_usb128_timeline.json').write_text(json.dumps(timeline,indent=2)+'\n')
    node.destroy_node();rclpy.shutdown()
