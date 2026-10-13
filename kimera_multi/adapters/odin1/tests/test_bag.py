import cv2
import numpy as np
import pytest
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

from odin1_adapter.bag import export_bag
from test_geometry import config


def recording(path,imu_gap=False):
    store=get_typestore(Stores.ROS2_HUMBLE)
    cls=store.types
    def header(t):
        return cls['std_msgs/msg/Header'](cls['builtin_interfaces/msg/Time'](t//10**9,t%10**9),'raw')
    def vec():
        return cls['geometry_msgs/msg/Vector3'](0.,0.,0.)
    def odom(t):
        point=cls['geometry_msgs/msg/Point'](0.,0.,0.)
        quat=cls['geometry_msgs/msg/Quaternion'](0.,0.,0.,1.)
        pose=cls['geometry_msgs/msg/PoseWithCovariance'](cls['geometry_msgs/msg/Pose'](point,quat),np.zeros(36))
        twist=cls['geometry_msgs/msg/TwistWithCovariance'](cls['geometry_msgs/msg/Twist'](vec(),vec()),np.zeros(36))
        return cls['nav_msgs/msg/Odometry'](header(t),'imu',pose,twist)
    with Writer(path,version=9) as writer:
        types={'/image':'sensor_msgs/msg/CompressedImage','/cloud':'sensor_msgs/msg/PointCloud2',
               '/imu':'sensor_msgs/msg/Imu','/odom':'nav_msgs/msg/Odometry','/high':'nav_msgs/msg/Odometry'}
        connections={k:writer.add_connection(k,v,typestore=store) for k,v in types.items()}
        def put(topic,t,msg):
            writer.write(connections[topic],t,store.serialize_cdr(msg,types[topic]))
        for t in range(1_000_000_000,1_352_500_000,2_500_000):
            put('/high',t,odom(t))
            if not imu_gap or not 1_100_000_000<t<1_200_000_000:
                imu=cls['sensor_msgs/msg/Imu'](header(t),cls['geometry_msgs/msg/Quaternion'](0.,0.,0.,1.),
                    np.zeros(9),vec(),np.zeros(9),vec(),np.zeros(9))
                put('/imu',t,imu)
        for t in (1_050_000_000,1_150_000_000):
            put('/odom',t,odom(t))
            ok,jpeg=cv2.imencode('.jpg',np.full((9,9,3),127,np.uint8));assert ok
            put('/image',t,cls['sensor_msgs/msg/CompressedImage'](header(t),'jpeg',jpeg.reshape(-1)))
            fields=[cls['sensor_msgs/msg/PointField'](name,offset,typ,1) for name,offset,typ in
                    [('x',0,7),('y',4,7),('z',8,7),('confidence',12,4),('offset_time',14,7)]]
            raw=bytearray(18)
            dtype=np.dtype({'names':['x','y','z','confidence','offset_time'],
                'formats':['<f4','<f4','<f4','<u2','<f4'],'offsets':[0,4,8,12,14],'itemsize':18})
            a=np.ndarray(1,dtype=dtype,buffer=raw)
            a['x']=0;a['y']=0;a['z']=2;a['confidence']=100;a['offset_time']=.02
            cloud=cls['sensor_msgs/msg/PointCloud2'](header(t+1_685_342),1,1,fields,False,18,18,np.frombuffer(raw,np.uint8),True)
            put('/cloud',t+1_685_342,cloud)


def bag_config():
    return config(deskew='odometry',robot_name='robot7',sync_tolerance_ms=5.,max_pose_gap_ms=10.,
                  input_image_topic='/image',input_cloud_topic='/cloud',input_imu_topic='/imu',
                  input_odom_topic='/odom',input_highfreq_odom_topic='/high')


def test_ros2_to_ros1_preserves_units_frames_and_rgbd_pair_stamps(tmp_path):
    source=tmp_path/'source';recording(source)
    result=export_bag(source,tmp_path/'converted.bag',bag_config())
    assert result['output_counts']['rgb']==2
    assert result['output_counts']['depth']==2
    store=get_typestore(Stores.ROS1_NOETIC)
    images={}
    with AnyReader([tmp_path/'converted.bag'],default_typestore=store) as reader:
        for conn,t,raw in reader.messages():
            msg=reader.deserialize(raw,conn.msgtype)
            assert t==msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            if conn.topic.endswith('/image_raw'):
                assert msg.header.frame_id=='robot7/camera_optical'
                images.setdefault(t,{})[msg.encoding]=msg
            if conn.topic.endswith('/imu'):
                assert msg.header.frame_id=='robot7/imu'
    for pair in images.values():
        assert set(pair)=={'bgr8','32FC1'}
        assert pair['bgr8'].width==pair['32FC1'].width==9
        depth=np.frombuffer(pair['32FC1'].data,'<f4').reshape(9,9)
        assert depth[4,4]==2.
    assert len(images)==2


def test_imu_gap_is_rejected_before_output_is_created(tmp_path):
    source=tmp_path/'source';recording(source,imu_gap=True)
    destination=tmp_path/'converted.bag'
    with pytest.raises(ValueError,match='IMU recording has'):
        export_bag(source,destination,bag_config())
    assert not destination.exists()
    assert not destination.with_name(destination.name+'.partial').exists()
