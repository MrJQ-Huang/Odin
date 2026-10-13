"""ROS 1 live input, using the same calibration and projection as ROS 2/export."""
import copy
import json
import os
import queue
import threading
import time
from collections import Counter, deque

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')

import cv2
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, PointCloud2
from std_msgs.msg import String

from .calibration import load_config
from .geometry import PoseSeries, PoseUnavailable, RgbdProjector, cloud_arrays
from .synchronization import PairSynchronizer, pose_entry, stamp_ns


class Ros1Adapter:
    def __init__(self, config):
        self.config = config
        cv2.setNumThreads(1)
        self.projector = RgbdProjector(config)
        self.sync = PairSynchronizer(int(config['sync_tolerance_ms'] * 1e6))
        self.lock = threading.RLock()
        self.poses = deque(maxlen=4000)
        self.stats = Counter()
        self.last = {}
        self.clock_invalid = False
        self.work = queue.Queue(maxsize=10)
        self.stop = threading.Event()
        self.prefix = '/' + config['robot_name'] + '/kimera'
        self.camera_frame = config['robot_name'] + '/camera_optical'
        self.imu_frame = config['robot_name'] + '/imu'
        self.pubs = {
            key: rospy.Publisher(self.prefix + suffix, typ, queue_size=size)
            for key, typ, suffix, size in [
                ('rgb', Image, '/rgb/image_raw', 5),
                ('depth', Image, '/depth/image_raw', 5),
                ('mask', Image, '/depth/measured_mask', 5),
                ('info', CameraInfo, '/rgb/camera_info', 5),
                ('imu', Imu, '/imu', 4000),
                ('reference', Odometry, '/reference/odometry', 1000),
                ('diagnostics', String, '/diagnostics', 5)]}
        # Publish SDK poses at their high frequency. Images need not wait for a
        # slower reference stream to bracket the exact sensor timestamp.
        self.subs = [
            rospy.Subscriber(config['input_image_topic'], CompressedImage,
                             lambda m: self.on_pair('image', m), queue_size=20,
                             buff_size=16 * 1024**2, tcp_nodelay=True),
            rospy.Subscriber(config['input_cloud_topic'], PointCloud2,
                             lambda m: self.on_pair('cloud', m), queue_size=20,
                             buff_size=32 * 1024**2, tcp_nodelay=True),
            rospy.Subscriber(config['input_imu_topic'], Imu, self.on_imu,
                             queue_size=4000, tcp_nodelay=True),
            rospy.Subscriber(config['input_highfreq_odom_topic'], Odometry,
                             self.on_pose, queue_size=4000, tcp_nodelay=True)]
        self.worker = threading.Thread(target=self.process, daemon=True)
        self.worker.start()
        self.timer = rospy.Timer(rospy.Duration(1), self.diagnostics)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('ROS1 Odin adapter ready: %s; SDK sensor timestamps preserved', self.prefix)

    def increasing(self, key, msg):
        stamp = stamp_ns(msg)
        old = self.last.get(key)
        if old is not None and stamp <= old:
            self.stats[key + '_non_increasing'] += 1
            if old - stamp > 100_000_000:
                self.clock_invalid = True
                rospy.logerr('SDK sensor clock reset: start a fresh graph segment')
            return False
        self.last[key] = stamp
        return not self.clock_invalid

    def on_pair(self, kind, msg):
        with self.lock:
            for pair in self.sync.add(kind, msg):
                try:
                    self.work.put_nowait(pair)
                except queue.Full:
                    self.stats['rgbd_worker_queue_full'] += 1

    def on_imu(self, msg):
        with self.lock:
            if not self.increasing('imu', msg):
                return
            msg.header.frame_id = self.imu_frame
            self.pubs['imu'].publish(msg)
            self.stats['imu_forwarded'] += 1

    def on_pose(self, msg):
        with self.lock:
            if not self.increasing('pose', msg):
                return
            self.poses.append(pose_entry(msg))
            self.stats['poses_received'] += 1
            out = copy.deepcopy(msg)
            out.header.frame_id = self.config['robot_name'] + '/sdk_odom'
            out.child_frame_id = self.imu_frame
            self.pubs['reference'].publish(out)
            self.stats['sdk_odometry_forwarded'] += 1

    def image(self, pixels, encoding, source):
        pixels = np.ascontiguousarray(pixels)
        msg = Image()
        msg.header = copy.deepcopy(source.header)
        msg.header.frame_id = self.camera_frame
        msg.height, msg.width = pixels.shape[:2]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = pixels.strides[0]
        msg.data = pixels.tobytes()
        return msg

    def process(self):
        while not self.stop.is_set():
            try:
                image, cloud = self.work.get(timeout=.1)
            except queue.Empty:
                continue
            try:
                if self.clock_invalid:
                    continue
                start = time.monotonic()
                arrays = cloud_arrays(cloud)
                deadline = start + 1
                while not self.stop.is_set():
                    try:
                        with self.lock:
                            poses = PoseSeries(list(self.poses)) if self.config['deskew'] == 'odometry' else None
                        depth, mask = self.projector.project(arrays, stamp_ns(cloud), stamp_ns(image), poses)
                        break
                    except PoseUnavailable:
                        if time.monotonic() >= deadline:
                            raise
                        self.stop.wait(.01)
                if self.stop.is_set():
                    continue
                bgr = cv2.imdecode(np.frombuffer(image.data, np.uint8), cv2.IMREAD_COLOR)
                rgb = self.projector.rectify(bgr)
                info = CameraInfo()
                info.header = copy.deepcopy(image.header)
                info.header.frame_id = self.camera_frame
                info.width, info.height = self.config['output_width'], self.config['output_height']
                info.distortion_model = 'plumb_bob'
                info.D = [0.] * 5
                info.K = self.projector.K.reshape(-1).tolist()
                info.R = np.eye(3).reshape(-1).tolist()
                projection = np.zeros((3, 4))
                projection[:, :3] = self.projector.K
                info.P = projection.reshape(-1).tolist()
                self.pubs['info'].publish(info)
                self.pubs['rgb'].publish(self.image(rgb, 'bgr8', image))
                self.pubs['depth'].publish(self.image(depth.astype('<f4'), '32FC1', image))
                self.pubs['mask'].publish(self.image(mask, 'mono8', image))
                with self.lock:
                    self.stats['rgbd_published'] += 1
                    self.stats['rgbd_processing_ms_last'] = (time.monotonic() - start) * 1000
            except PoseUnavailable as error:
                with self.lock:
                    self.stats['rgbd_pose_unavailable'] += 1
                rospy.logwarn_throttle(5, str(error))
            except Exception as error:
                with self.lock:
                    self.stats['rgbd_errors'] += 1
                rospy.logerr_throttle(5, 'RGB-D conversion failed: ' + str(error))
            finally:
                self.work.task_done()

    def diagnostics(self, _):
        with self.lock:
            data = {'status': 'clock_reset' if self.clock_invalid else 'running',
                    'middleware': 'ROS1', 'deskew': self.config['deskew'],
                    'counters': dict(self.stats), 'synchronization': dict(self.sync.stats),
                    'worker_queue': self.work.qsize(), 'last_input_sensor_stamps_ns': dict(self.last)}
        self.pubs['diagnostics'].publish(String(data=json.dumps(data)))

    def shutdown(self):
        self.stop.set()
        self.worker.join(timeout=2)


def main():
    rospy.init_node('odin1_ros1_adapter')
    Ros1Adapter(load_config(rospy.get_param('~config')))
    rospy.spin()


if __name__ == '__main__':
    main()
