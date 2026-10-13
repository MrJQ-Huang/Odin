"""Convert Odin ROS 2 recordings to registered, deskewed ROS 1 RGB-D bags."""
import argparse
import copy
import json
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore

from .calibration import load_config
from .geometry import PoseSeries, PoseUnavailable, RgbdProjector, cloud_arrays
from .synchronization import PairSynchronizer, pose_entry, stamp_ns


def export_bag(source, destination, config, start=0., duration=None, allow_imu_gaps=False, compression='none'):
    if compression not in ('none', 'bz2'):
        raise ValueError('ROS 1 export compression must be none or bz2')
    if start < 0 or duration is not None and duration <= 0:
        raise ValueError('start must be nonnegative and duration must be positive')
    source,destination = Path(source),Path(destination)
    report_path = destination.with_suffix('.json')
    preview_path = destination.with_suffix('.preview.png')
    partial = destination.with_name(destination.name+'.partial')
    for path in (destination,report_path,preview_path,partial):
        if path.exists():
            raise ValueError(f'Refusing to overwrite {path}')
    destination.parent.mkdir(parents=True,exist_ok=True)
    input_store = get_typestore(Stores.ROS2_HUMBLE)
    store = get_typestore(Stores.ROS1_NOETIC)
    prefix = '/'+config['robot_name']+'/kimera'
    topics = {'rgb':prefix+'/rgb/image_raw','depth':prefix+'/depth/image_raw',
              'camera_info':prefix+'/rgb/camera_info','mask':prefix+'/depth/measured_mask',
              'imu':prefix+'/imu','reference':prefix+'/reference/odometry'}
    frames = {'camera':config['robot_name']+'/camera_optical','imu':config['robot_name']+'/imu',
              'odom':config['robot_name']+'/odin_odom'}
    image_topic,cloud_topic = config['input_image_topic'],config['input_cloud_topic']
    imu_topic,odom_topic = config['input_imu_topic'],config['input_odom_topic']
    high_topic = config['input_highfreq_odom_topic']
    expected = {image_topic:'sensor_msgs/msg/CompressedImage',cloud_topic:'sensor_msgs/msg/PointCloud2',
                imu_topic:'sensor_msgs/msg/Imu',odom_topic:'nav_msgs/msg/Odometry'}
    if config['deskew'] == 'odometry':
        expected[high_topic] = 'nav_msgs/msg/Odometry'
    pose_rows,imu_times,image_times = [],[],[]
    with AnyReader([source],default_typestore=input_store) as reader:
        available = {c.topic:c.msgtype for c in reader.connections}
        if any(available.get(topic) != typ for topic,typ in expected.items()):
            raise ValueError(f'Missing or incorrectly typed input topics: {expected}; available: {available}')
        connections = [c for c in reader.connections if c.topic in (image_topic,imu_topic,high_topic)]
        last = {}
        for connection,_,raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw,connection.msgtype)
            time = stamp_ns(msg)
            if connection.topic in last and time <= last[connection.topic]:
                raise ValueError(f'Non-increasing sensor clock on {connection.topic}; split recordings at resets')
            last[connection.topic] = time
            if connection.topic == image_topic:
                image_times.append(time)
            elif connection.topic == imu_topic:
                imu_times.append(time)
            elif connection.topic == high_topic:
                pose_rows.append(pose_entry(msg))
    if not image_times or not imu_times:
        raise ValueError('Recording has no RGB or IMU messages')
    lower = image_times[0]+int(start*1e9)
    upper = image_times[-1]+1 if duration is None else lower+int(duration*1e9)
    # Include IMU context around image limits for estimator preintegration.
    imu_window = np.asarray([t for t in imu_times if lower-200_000_000 <= t <= upper+200_000_000],np.int64)
    selected_images = [t for t in image_times if lower<=t<upper]
    if not selected_images or len(imu_window)<2:
        raise ValueError('Requested interval does not contain RGB and IMU data')
    max_imu_gap_ms = float(np.max(np.diff(imu_window))*1e-6)
    if max_imu_gap_ms > 20 and not allow_imu_gaps:
        raise ValueError(f'IMU recording has a {max_imu_gap_ms:.1f} ms gap. Choose a continuous interval or explicitly pass --allow-imu-gaps for diagnostics.')
    poses = PoseSeries(pose_rows) if config['deskew']=='odometry' else None
    projector = RgbdProjector(config)
    sync = PairSynchronizer(int(config['sync_tolerance_ms']*1e6))
    counts,skips = Counter(),Counter()
    offsets,coverage,direct_coverage = [],[],[]
    first_preview = None
    Header,Time = store.types['std_msgs/msg/Header'],store.types['builtin_interfaces/msg/Time']
    Image,Imu = store.types['sensor_msgs/msg/Image'],store.types['sensor_msgs/msg/Imu']
    CameraInfo,ROI = store.types['sensor_msgs/msg/CameraInfo'],store.types['sensor_msgs/msg/RegionOfInterest']
    Odometry = store.types['nav_msgs/msg/Odometry']

    def header(time,frame):
        return Header(seq=0,stamp=Time(sec=time//1_000_000_000,nanosec=time%1_000_000_000),frame_id=frame)

    writer = Writer(partial)
    # rosbags' LZ4 framing can be read by rosbags itself but was rejected by
    # Noetic's native roslz4 player. Use native-compatible none / BZ2 only.
    if compression == 'bz2':
        writer.set_compression(Writer.CompressionFormat.BZ2)
    try:
        with writer, AnyReader([source],default_typestore=input_store) as reader:
            msg_types = {'rgb':Image.__msgtype__,'depth':Image.__msgtype__,'mask':Image.__msgtype__,
                         'camera_info':CameraInfo.__msgtype__,'imu':Imu.__msgtype__,'reference':Odometry.__msgtype__}
            connections = {key:writer.add_connection(topics[key],typ,typestore=store) for key,typ in msg_types.items()}

            def write(key,time,msg):
                writer.write(connections[key],time,store.serialize_ros1(msg,msg_types[key]))
                counts[key] += 1

            def write_image(key,time,array,encoding):
                array = np.ascontiguousarray(array)
                write(key,time,Image(header=header(time,frames['camera']),height=array.shape[0],width=array.shape[1],
                    encoding=encoding,is_bigendian=0,step=array.strides[0],data=array.view(np.uint8).reshape(-1)))

            wanted = [c for c in reader.connections if c.topic in (image_topic,cloud_topic,imu_topic,odom_topic)]
            for connection,_,raw in reader.messages(connections=wanted):
                msg = reader.deserialize(raw,connection.msgtype)
                time = stamp_ns(msg)
                if connection.topic==imu_topic:
                    if lower-200_000_000<=time<=upper+200_000_000:
                        write('imu',time,Imu(header(time,frames['imu']),msg.orientation,msg.orientation_covariance,
                            msg.angular_velocity,msg.angular_velocity_covariance,msg.linear_acceleration,msg.linear_acceleration_covariance))
                    continue
                if not lower<=time<upper:
                    # Keep clouds immediately around the RGB boundaries for pairing.
                    if connection.topic != cloud_topic or not lower-sync.tolerance<=time<upper+sync.tolerance:
                        continue
                if connection.topic==odom_topic:
                    # WIWC matrices may occupy covariance fields: not uncertainty estimates.
                    msg = copy.deepcopy(msg)
                    msg.pose.covariance = np.zeros(36)
                    msg.twist.covariance = np.zeros(36)
                    write('reference',time,Odometry(header(time,frames['odom']),frames['imu'],msg.pose,msg.twist))
                    continue
                kind = 'image' if connection.topic==image_topic else 'cloud'
                for image,cloud in sync.add(kind,msg):
                    time = stamp_ns(image)
                    try:
                        depth,mask = projector.project(cloud_arrays(cloud),stamp_ns(cloud),time,poses)
                    except PoseUnavailable as exc:
                        skips[str(exc)] += 1
                        continue
                    bgr = cv2.imdecode(np.asarray(image.data,np.uint8),cv2.IMREAD_COLOR)
                    rgb = projector.rectify(bgr)
                    write_image('rgb',time,rgb,'bgr8')
                    write_image('depth',time,depth.astype('<f4'),'32FC1')
                    write_image('mask',time,mask,'mono8')
                    P = np.zeros((3,4)); P[:,:3]=projector.K
                    write('camera_info',time,CameraInfo(header(time,frames['camera']),config['output_height'],config['output_width'],
                        'plumb_bob',np.zeros(5),projector.K.reshape(-1),np.eye(3).reshape(-1),P.reshape(-1),
                        0,0,ROI(0,0,0,0,False)))
                    offsets.append((stamp_ns(cloud)-time)*1e-6)
                    coverage.append(float((depth>0).mean())); direct_coverage.append(float((mask>0).mean()))
                    if first_preview is None:
                        color = cv2.applyColorMap(np.uint8(np.clip(depth,0,config['max_depth_m'])/config['max_depth_m']*255),cv2.COLORMAP_TURBO)
                        color[depth==0]=0
                        first_preview = np.concatenate([rgb,color],axis=1)
                    if counts['rgb'] % 100 == 0:
                        print(f"Converted {counts['rgb']} RGB-D pairs",flush=True)
        if not counts['rgb']:
            raise ValueError(f'No usable RGB-D frames; pose skips: {dict(skips)}; sync: {dict(sync.stats)}')
        result = {'source':str(source.resolve()),'destination':str(destination.resolve()),
            'compression':compression,
            'clock':'Device header timestamps preserved; bag event timestamps use the same clock',
            'deskew':config['deskew'],'T_imu_lidar':config['T_imu_lidar'].flatten().tolist(),
            'input_image_count_in_interval':len(selected_images),'output_counts':dict(counts),
            'sync':dict(sync.stats),'frames_skipped_for_pose_coverage':dict(skips),
            'unpaired_at_end':{k:len(v) for k,v in sync.queues.items()},
            'max_input_imu_gap_ms':max_imu_gap_ms,'imu_gap_override':allow_imu_gaps,
            'cloud_minus_rgb_ms':{'min':min(offsets),'max':max(offsets)},
            'positive_depth_fraction_mean':float(np.mean(coverage)),
            'direct_measured_pixel_fraction_mean':float(np.mean(direct_coverage)),
            'note':'Splatted depth pixels are not independent measurements. Reference odometry is not ground truth and is not enabled as an external VIO factor.'}
        report_path.write_text(json.dumps(result,indent=2))
        cv2.imwrite(str(preview_path),first_preview)
        partial.rename(destination)
        return result
    except Exception:
        partial.unlink(missing_ok=True)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--start',type=float,default=0.,help='Seconds after first RGB sensor timestamp')
    parser.add_argument('--duration',type=float)
    parser.add_argument('--allow-imu-gaps',action='store_true',help='Diagnostic exports only; do not treat as valid VIO input')
    parser.add_argument('--compression',choices=['none','bz2'],default='none',help='Native ROS 1 compatible; none favors replay speed')
    args = parser.parse_args()
    print(json.dumps(export_bag(args.input,args.output,load_config(args.config),args.start,args.duration,args.allow_imu_gaps,args.compression),indent=2))


if __name__=='__main__':
    main()
