#!/usr/bin/python3
"""Measure native depth output; defer file/image processing until sampling ends."""
import json
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, PointCloud2

from probe import stamp, percentiles


root = Path(__file__).resolve().parent / 'native_depth'
root.mkdir(exist_ok=True)
rclpy.init()
node = Node('odin_native_depth_probe')
streams = {'depth_img_competetion': Image, 'image/undistorted': Image,
           'cloud_raw': PointCloud2, 'imu': Imu}
samples = {key: [] for key in streams}
images = deque(maxlen=6)
depths = deque(maxlen=6)
start = time.monotonic()


def callback(key, msg):
    samples[key].append([time.monotonic() - start, stamp(msg)])
    if key == 'image/undistorted':
        images.append(msg)
    elif key == 'depth_img_competetion':
        depths.append(msg)


subscriptions = [node.create_subscription(typ, '/odin1/' + key,
                 lambda msg, key=key: callback(key, msg), qos_profile_sensor_data)
                 for key, typ in streams.items()]
try:
    while time.monotonic() - start < 30:
        rclpy.spin_once(node, timeout_sec=.1)
finally:
    result = {'duration_s': time.monotonic() - start, 'topics': {}}
    for key, rows in samples.items():
        a = np.asarray(rows)
        info = {'count': len(rows)}
        if len(rows) > 1:
            dt = np.diff(a[:, 1])
            info.update(arrival_hz=float((len(rows)-1)/(a[-1,0]-a[0,0])),
                        stamp_hz=float((len(rows)-1)/(a[-1,1]-a[0,1])),
                        stamp_dt_ms=percentiles(dt*1000),
                        non_increasing_stamps=int((dt <= 0).sum()))
        result['topics'][key] = info
        np.savetxt(root / (key.replace('/', '_')+'_timestamps.csv'), a,
                   delimiter=',', header='arrival_elapsed_s,header_stamp_s')
    if depths and images:
        d = depths[-1]
        rgb = min(images, key=lambda x: abs(stamp(x)-stamp(d)))
        endian = '>' if d.is_bigendian else '<'
        depth = np.ndarray((d.height,d.width), dtype=endian+'f4',
                           buffer=d.data, strides=(d.step,4)).copy()
        img = np.frombuffer(rgb.data,np.uint8).reshape(rgb.height,rgb.step)
        img = img[:,:rgb.width*3].reshape(rgb.height,rgb.width,3).copy()
        if rgb.encoding == 'rgb8':
            img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
        valid = np.isfinite(depth) & (depth > .1)
        result['depth_sample'] = dict(width=d.width,height=d.height,encoding=d.encoding,
            frame_id=d.header.frame_id, rgb_frame_id=rgb.header.frame_id,
            depth_minus_rgb_ms=(stamp(d)-stamp(rgb))*1000,
            positive_finite_fraction=float(valid.mean()),depth_m=percentiles(depth[valid]))
        keys = cv2.ORB_create(2000).detect(cv2.cvtColor(img,cv2.COLOR_BGR2GRAY),None)
        coords = np.asarray([key.pt for key in keys],int)
        result['depth_sample']['orb_keypoints'] = len(keys)
        result['depth_sample']['orb_positive_depth'] = int(valid[coords[:,1],coords[:,0]].sum()) if keys else 0
        np.save(root/'depth_sample.npy',depth)
        cv2.imwrite(str(root/'rgb_sample.png'),img)
        color = cv2.applyColorMap(np.uint8(np.clip(depth,0,8)/8*255),cv2.COLORMAP_TURBO)
        color[~valid]=0
        cv2.imwrite(str(root/'depth_preview.png'),color)
        cv2.imwrite(str(root/'rgb_depth_preview.png'),np.concatenate([
            cv2.resize(img,(800,648)),cv2.resize(color,(800,648))],axis=1))
    (root/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    print(json.dumps(result,indent=2,allow_nan=False),flush=True)
    node.destroy_node()
    rclpy.shutdown()
