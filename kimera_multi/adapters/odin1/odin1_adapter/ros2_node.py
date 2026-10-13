"""Live Odin adapter. Keep IMU forwarding separate from RGB-D processing."""
import argparse
import array
import copy
import json
import os
import queue
import threading
import time
from collections import Counter, deque

# Set before OpenCV/NumPy load their native thread pools. Tiny per-frame
# matrices must not compete with the two 400 Hz ROS callbacks for every core.
os.environ['OPENBLAS_NUM_THREADS'] = os.environ.get('ODIN1_BLAS_THREADS','1')
os.environ['OMP_NUM_THREADS'] = os.environ.get('ODIN1_BLAS_THREADS','1')

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from .calibration import load_config
from .geometry import PoseSeries, PoseUnavailable, RgbdProjector, cloud_arrays
from .synchronization import PairSynchronizer, pose_entry, stamp_ns


def reliable(depth):
    return QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=depth,reliability=ReliabilityPolicy.RELIABLE)


class OdinAdapter(Node):
    def __init__(self, config):
        super().__init__('odin1_kimera_adapter')
        self.config = config
        cv2.setNumThreads(1)
        self.projector = RgbdProjector(config)
        self.sync = PairSynchronizer(int(config['sync_tolerance_ms']*1e6))
        self.poses = deque(maxlen=4000)
        self.lock = threading.Lock()
        self.stats = Counter()
        self.last = {}
        self.clock_invalid = False
        # About one second at the configured camera rate, to absorb USB/DDS
        # bursts without letting an unbounded image backlog grow.
        self.work = queue.Queue(maxsize=10)
        self.stop_event = threading.Event()
        self.prefix = '/'+config['robot_name']+'/kimera'
        self.camera_frame = config['robot_name']+'/camera_optical'
        self.imu_frame = config['robot_name']+'/imu'
        self.pubs = {key:self.create_publisher(typ,self.prefix+suffix,reliable(depth))
                     for key,typ,suffix,depth in [('rgb',Image,'/rgb/image_raw',5),
                       ('depth',Image,'/depth/image_raw',5),('mask',Image,'/depth/measured_mask',5),
                       ('info',CameraInfo,'/rgb/camera_info',5),('imu',Imu,'/imu',4000),
                       ('reference',Odometry,'/reference/odometry',20),('diagnostics',String,'/diagnostics',5)]}
        self.sensor_group = MutuallyExclusiveCallbackGroup()
        self.imu_group = MutuallyExclusiveCallbackGroup()
        self.pose_group = MutuallyExclusiveCallbackGroup()
        self.subs = [self.create_subscription(CompressedImage,config['input_image_topic'],
                     lambda msg:self.on_pair('image',msg),reliable(20),callback_group=self.sensor_group),
                     self.create_subscription(PointCloud2,config['input_cloud_topic'],
                     lambda msg:self.on_pair('cloud',msg),reliable(20),callback_group=self.sensor_group),
                     self.create_subscription(Imu,config['input_imu_topic'],self.on_imu,reliable(4000),callback_group=self.imu_group),
                     self.create_subscription(Odometry,config['input_odom_topic'],self.on_reference,reliable(20),callback_group=self.pose_group)]
        if config['deskew']=='odometry':
            self.subs.append(self.create_subscription(Odometry,config['input_highfreq_odom_topic'],
                self.on_pose,reliable(4000),callback_group=self.pose_group))
        self.timer = self.create_timer(1.,self.diagnostics)
        self.worker = threading.Thread(target=self.process,name='odin_rgbd_worker',daemon=True)
        self.worker.start()
        self.get_logger().info(f'Publishing {self.prefix}; deskew={config["deskew"]}. Preserve device timestamps; reference odometry is not a VIO factor.')

    def increasing(self,key,msg):
        timestamp = stamp_ns(msg)
        previous = self.last.get(key)
        if previous is not None and timestamp<=previous:
            self.stats[key+'_non_increasing'] += 1
            if previous-timestamp>100_000_000:
                self.clock_invalid = True
                self.get_logger().error('Device clock reset detected. Restart adapter and estimator after resetting recording state.')
            return False
        self.last[key]=timestamp
        return not self.clock_invalid

    def on_pair(self,kind,msg):
        for pair in self.sync.add(kind,msg):
            try:
                self.work.put_nowait(pair)
            except queue.Full:
                self.stats['rgbd_worker_queue_full'] += 1

    def on_imu(self,msg):
        if self.increasing('imu',msg):
            msg.header.frame_id=self.imu_frame
            self.pubs['imu'].publish(msg)
            self.stats['imu_forwarded'] += 1

    def on_pose(self,msg):
        if self.increasing('pose',msg):
            with self.lock:
                self.poses.append(pose_entry(msg))
            self.stats['poses_received'] += 1

    def on_reference(self,msg):
        if self.increasing('reference',msg):
            outgoing=copy.deepcopy(msg)
            outgoing.header.frame_id=self.config['robot_name']+'/odin_odom'
            outgoing.child_frame_id=self.imu_frame
            outgoing.pose.covariance=[0.]*36
            outgoing.twist.covariance=[0.]*36
            self.pubs['reference'].publish(outgoing)
            self.stats['reference_forwarded'] += 1

    def image_message(self,pixels,encoding,source):
        pixels=np.ascontiguousarray(pixels)
        msg=Image()
        msg.header=copy.deepcopy(source.header)
        msg.header.frame_id=self.camera_frame
        msg.height,msg.width=pixels.shape[:2]
        msg.encoding=encoding; msg.is_bigendian=0
        msg.step=pixels.strides[0]
        # The generated ROS setter validates ordinary bytes element by element.
        # A typed byte array takes its validated bulk path.
        msg.data=array.array('B',pixels.tobytes())
        return msg

    def process(self):
        while not self.stop_event.is_set():
            try:
                image,cloud=self.work.get(timeout=.1)
            except queue.Empty:
                continue
            try:
                if self.clock_invalid:
                    continue
                processing_start=time.monotonic()
                arrays=cloud_arrays(cloud)
                deadline=time.monotonic()+1.
                while not self.stop_event.is_set():
                    try:
                        with self.lock:
                            poses=PoseSeries(list(self.poses)) if self.config['deskew']=='odometry' else None
                        depth,mask=self.projector.project(arrays,stamp_ns(cloud),stamp_ns(image),poses)
                        break
                    except PoseUnavailable:
                        if time.monotonic()>=deadline:
                            raise
                        self.stop_event.wait(.01)
                if self.stop_event.is_set():
                    continue
                bgr=cv2.imdecode(np.frombuffer(image.data,np.uint8),cv2.IMREAD_COLOR)
                rgb=self.projector.rectify(bgr)
                self.pubs['rgb'].publish(self.image_message(rgb,'bgr8',image))
                self.pubs['depth'].publish(self.image_message(depth.astype('<f4'),'32FC1',image))
                self.pubs['mask'].publish(self.image_message(mask,'mono8',image))
                info=CameraInfo()
                info.header=copy.deepcopy(image.header); info.header.frame_id=self.camera_frame
                info.width,info.height=self.config['output_width'],self.config['output_height']
                info.distortion_model='plumb_bob'; info.d=[0.]*5
                info.k=self.projector.K.reshape(-1).tolist(); info.r=np.eye(3).reshape(-1).tolist()
                P=np.zeros((3,4)); P[:,:3]=self.projector.K; info.p=P.reshape(-1).tolist()
                self.pubs['info'].publish(info)
                self.stats['rgbd_published'] += 1
                self.stats['rgbd_processing_ms_last']=(time.monotonic()-processing_start)*1000
            except PoseUnavailable as exc:
                self.stats['rgbd_pose_unavailable'] += 1
                self.get_logger().warning(str(exc))
            except Exception as exc:
                self.stats['rgbd_errors'] += 1
                self.get_logger().error(f'RGB-D conversion failed: {exc}')
            finally:
                self.work.task_done()

    def diagnostics(self):
        msg=String()
        msg.data=json.dumps({'status':'clock_reset' if self.clock_invalid else 'running',
            'deskew':self.config['deskew'],'counters':dict(self.stats),'synchronization':dict(self.sync.stats),
            'worker_queue':self.work.qsize(),'last_input_sensor_stamps_ns':dict(self.last)})
        self.pubs['diagnostics'].publish(msg)

    def destroy_node(self):
        self.stop_event.set()
        self.worker.join(timeout=2.)
        return super().destroy_node()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',required=True)
    args,ros_args=parser.parse_known_args()
    config=load_config(args.config)
    # Keep the context alive until the image worker stops during SIGINT.
    rclpy.init(args=ros_args,signal_handler_options=SignalHandlerOptions.NO)
    node=OdinAdapter(config)
    # Callbacks only enqueue data or forward small messages. The image worker
    # is separate, avoiding rclpy thread-pool overhead on the two 400 Hz inputs.
    executor=SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
