#!/usr/bin/python3
"""Observe Odin ROS 2 streams without publishing device commands."""
import argparse
import csv
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, PointCloud2
from nav_msgs.msg import Odometry


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def percentiles(values):
    a = np.asarray(values, dtype=float)
    return dict(zip(['min', 'p50', 'p95', 'max'], np.percentile(a, [0, 50, 95, 100]).tolist())) if a.size else None


class Probe(Node):
    def __init__(self, output, selected=None, snapshots=True):
        super().__init__('odin1_stream_probe')
        self.output = output
        self.start = time.monotonic()
        self.samples = {}
        self.details = {}
        self.poses = {}
        self.imus = []
        self.snapshots = set()
        self.errors = {}
        self.save_images = snapshots
        streams = {'image': Image, 'image/undistorted': Image, 'imu': Imu,
                   'cloud_raw': PointCloud2, 'odometry': Odometry,
                   'odometry_highfreq': Odometry, 'depth_img_competetion': Image}
        if selected:
            streams = {key: typ for key, typ in streams.items() if key in selected}
        self.subs = [self.create_subscription(typ, '/odin1/' + key,
                     lambda msg, key=key: self.callback(key, msg), qos_profile_sensor_data)
                     for key, typ in streams.items()]

    def callback(self, key, msg):
        arrival = time.monotonic() - self.start
        self.samples.setdefault(key, []).append((arrival, stamp(msg)))
        try:
            if isinstance(msg, Odometry):
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                self.poses.setdefault(key, []).append([arrival, stamp(msg), p.x, p.y, p.z, q.x, q.y, q.z, q.w])
                self.details[key] = {'frame_id': msg.header.frame_id, 'child_frame_id': msg.child_frame_id,
                                     'pose_covariance': list(msg.pose.covariance),
                                     'twist_covariance': list(msg.twist.covariance)}
            elif isinstance(msg, Imu):
                a, g = msg.linear_acceleration, msg.angular_velocity
                self.imus.append([arrival, stamp(msg), a.x, a.y, a.z, g.x, g.y, g.z])
                self.details[key] = {'frame_id': msg.header.frame_id,
                                     'orientation_covariance': list(msg.orientation_covariance)}
            elif isinstance(msg, Image):
                self.details[key] = {'width': msg.width, 'height': msg.height, 'encoding': msg.encoding,
                                     'frame_id': msg.header.frame_id, 'step': msg.step}
                snapshot_key = (key, int(arrival // 10))
                if self.save_images and key in ('image', 'image/undistorted') and snapshot_key not in self.snapshots:
                    self.snapshots.add(snapshot_key)
                    raw = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
                    img = raw[:, :msg.width * 3].reshape(msg.height, msg.width, 3).copy()
                    if msg.encoding == 'rgb8':
                        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    filename = key.replace('/', '_') + '_%02d.png' % snapshot_key[1]
                    cv2.imwrite(str(self.output / filename), img)
            elif isinstance(msg, PointCloud2) and key not in self.snapshots:
                self.snapshots.add(key)
                types = {1:'i1', 2:'u1', 3:'i2', 4:'u2', 5:'i4', 6:'u4', 7:'f4', 8:'f8'}
                endian = '>' if msg.is_bigendian else '<'
                dtype = np.dtype({'names':[f.name for f in msg.fields],
                                  'formats':[endian + types[f.datatype] for f in msg.fields],
                                  'offsets':[f.offset for f in msg.fields], 'itemsize':msg.point_step})
                rows = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=bytes(msg.data),
                                  strides=(msg.row_step, msg.point_step)).reshape(-1)
                arrays = {name: rows[name] for name in dtype.names}
                np.savez_compressed(self.output / 'cloud_raw_sample.npz', **arrays)
                xyz = np.stack([arrays[x] for x in ('x', 'y', 'z')], axis=-1)
                distances = np.linalg.norm(xyz, axis=1)
                valid = np.isfinite(xyz).all(axis=1) & (distances > 0.05)
                self.details[key] = {'frame_id':msg.header.frame_id, 'width':msg.width, 'height':msg.height,
                                     'point_step':msg.point_step, 'fields':list(dtype.names),
                                     'valid_fraction':float(valid.mean()),
                                     'range_m':percentiles(distances[valid])}
        except Exception as exc:
            self.errors[key] = str(exc)

    def report(self):
        duration = time.monotonic() - self.start
        result = {'duration_s':duration, 'topics':{}, 'callback_errors':self.errors}
        for key, entries in self.samples.items():
            a = np.asarray(entries)
            dt_stamp, dt_arrival = np.diff(a[:,1]), np.diff(a[:,0])
            positive = dt_stamp[dt_stamp > 0]
            result['topics'][key] = {'count':len(entries),
                'arrival_hz':float((len(entries)-1)/(a[-1,0]-a[0,0])) if len(entries)>1 else None,
                'stamp_hz':float((len(entries)-1)/(a[-1,1]-a[0,1])) if len(entries)>1 and a[-1,1]>a[0,1] else None,
                'stamp_dt_ms':percentiles(dt_stamp * 1000),
                'arrival_dt_ms':percentiles(dt_arrival * 1000),
                'non_increasing_stamps':int((dt_stamp<=0).sum()),
                'gaps_over_3_median':int((dt_stamp > 3 * np.median(positive)).sum()) if positive.size else 0,
                'details':self.details.get(key)}
            with (self.output / (key.replace('/', '_') + '_timestamps.csv')).open('w') as f:
                writer=csv.writer(f); writer.writerow(['arrival_elapsed_s','header_stamp_s']); writer.writerows(entries)
        for key, entries in self.poses.items():
            a = np.asarray(entries); xyz=a[:,2:5]
            q=a[:,5:9]; q=q/np.linalg.norm(q,axis=1,keepdims=True)
            angles=np.degrees(2*np.arccos(np.clip(np.abs(q@q[0]),0,1)))
            result['topics'][key]['motion'] = {'end_displacement_m':float(np.linalg.norm(xyz[-1]-xyz[0])),
                'max_displacement_m':float(np.linalg.norm(xyz-xyz[0],axis=1).max()),
                'position_axis_span_m':np.ptp(xyz,axis=0).tolist(),
                'max_rotation_from_first_deg':float(angles.max()),
                'max_consecutive_position_step_m':float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).max()) if len(xyz)>1 else 0}
            with (self.output / (key+'_poses.csv')).open('w') as f:
                writer=csv.writer(f); writer.writerow(['arrival_s','stamp_s','x','y','z','qx','qy','qz','qw']); writer.writerows(entries)
        if self.imus:
            a=np.asarray(self.imus)
            result['imu_values']={'accel_mean_m_s2':a[:,2:5].mean(axis=0).tolist(),
                'accel_std_m_s2':a[:,2:5].std(axis=0).tolist(),
                'gyro_mean_rad_s':a[:,5:8].mean(axis=0).tolist(),
                'gyro_std_rad_s':a[:,5:8].std(axis=0).tolist(),
                'accel_norm_m_s2':percentiles(np.linalg.norm(a[:,2:5],axis=1))}
            with (self.output/'imu_values.csv').open('w') as f:
                writer=csv.writer(f); writer.writerow(['arrival_s','stamp_s','ax','ay','az','gx','gy','gz']); writer.writerows(self.imus)
        for left,right in [('image','cloud_raw'),('image','image/undistorted'),('image','depth_img_competetion')]:
            if left not in self.samples or right not in self.samples:
                continue
            a=np.asarray(self.samples[left])[:,1]; b=np.sort(np.asarray(self.samples[right])[:,1])
            idx=np.searchsorted(b,a); idx0=np.clip(idx-1,0,len(b)-1); idx1=np.clip(idx,0,len(b)-1)
            signed=np.where(np.abs(b[idx0]-a)<=np.abs(b[idx1]-a),b[idx0]-a,b[idx1]-a)
            result.setdefault('nearest_stamp_offsets_ms',{})[left+'->'+right]=percentiles(signed*1000)
        (self.output/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False))
        print(json.dumps(result,indent=2,allow_nan=False),flush=True)


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--duration',type=float,default=30); parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--streams',nargs='+'); parser.add_argument('--no-images',action='store_true')
    args=parser.parse_args(); args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init(); node=Probe(args.output,args.streams,not args.no_images)
    try:
        while time.monotonic()-node.start<args.duration:
            rclpy.spin_once(node,timeout_sec=0.1)
    finally:
        node.report(); node.destroy_node(); rclpy.shutdown()


if __name__=='__main__':
    main()
