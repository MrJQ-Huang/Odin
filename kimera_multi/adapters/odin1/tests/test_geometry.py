from types import SimpleNamespace as NS

import numpy as np
import pytest

from odin1_adapter.geometry import PoseSeries, PoseUnavailable, RgbdProjector, cloud_arrays
from odin1_adapter.synchronization import PairSynchronizer


def config(**changes):
    c = dict(camera=dict(cam_model='FishPoly',image_width=9,image_height=9,A11=4.,A22=4.,A12=0.,
                        u0=4.,v0=4.,**{f'k{i}':0. for i in range(2,8)}),
             output_width=9,output_height=9,T_camera_lidar=np.eye(4),T_imu_lidar=np.eye(4),
             min_depth_m=.3,max_depth_m=8.,confidence_threshold=35,splat_radius=0,
             edge_threshold_m=100.,deskew='none',max_pose_gap_ms=200.)
    c.update(changes)
    return c


def cloud(points,confidence=None,offsets=None):
    points=np.asarray(points,float)
    result={key:points[:,i] for i,key in enumerate(('x','y','z'))}
    result['confidence']=np.full(len(points),100) if confidence is None else np.asarray(confidence)
    if offsets is not None:
        result['offset_time']=np.asarray(offsets)
    return result


def test_projection_uses_optical_z_and_closest_surface():
    p=RgbdProjector(config())
    # Both points map to pixel (6, 4); the nearer Z must win regardless of input order.
    for points in ([[2,0,4],[1,0,2]],[[1,0,2],[2,0,4]]):
        depth,mask=p.project(cloud(points),0,0)
        assert depth[4,6]==2
        assert mask[4,6]==255


def test_invalid_confidence_range_and_nan_do_not_supply_depth():
    p=RgbdProjector(config())
    depth,_=p.project(cloud([[0,0,2],[0,0,-2],[0,0,9],[np.nan,0,2]],confidence=[34,100,100,100]),0,0)
    assert not depth.any()


def test_optical_axis_rectification_maps_to_principal_point():
    p=RgbdProjector(config())
    assert p.map_x[4,4]==4
    assert p.map_y[4,4]==4
    assert np.isfinite(p.map_x).all()


def test_splat_does_not_extend_outside_camera_fov():
    p=RgbdProjector(config(splat_radius=1))
    depth,_=p.project(cloud([[3,0,2]]),0,0)
    assert not depth.any()


def test_depth_edges_are_removed_without_filling_large_holes():
    p=RgbdProjector(config(edge_threshold_m=.2,splat_radius=1))
    depth,_=p.project(cloud([[0,0,2],[1,0,4]]),0,0)
    assert depth[4,5]==0  # Mixed-depth boundary.
    assert depth[0,0]==0  # No global completion of missing depth.


def test_translation_deskew_uses_each_points_acquisition_time():
    poses=PoseSeries([(0,[0,0,0],[0,0,0,1]),(100_000_000,[1,0,0],[0,0,0,1])])
    p=RgbdProjector(config(deskew='odometry'))
    depth,_=p.project(cloud([[0,0,2]],offsets=[.1]),0,0,poses)
    assert depth[4,6]==2  # IMU translated +1 m between image and point time.
    assert depth[4,4]==0


def test_pose_rotation_uses_shortest_arc_and_no_extrapolation():
    q=[0,0,np.sin(np.pi/4),np.cos(np.pi/4)]
    poses=PoseSeries([(0,[0,0,0],[0,0,0,1]),(100_000_000,[0,0,0],(-np.asarray(q)).tolist())])
    R,_=poses.interpolate([50_000_000],200_000_000)
    np.testing.assert_allclose(R[0]@[1,0,0],[2**-.5,2**-.5,0],atol=1e-6)
    with pytest.raises(PoseUnavailable):
        poses.interpolate([100_000_001],200_000_000)
    with pytest.raises(PoseUnavailable):
        poses.interpolate([50_000_000],10_000_000)


def test_cloud_parser_honors_big_endian_unaligned_fields_and_row_padding():
    dtype=np.dtype({'names':['x','y','z','confidence'],
                    'formats':['>f4','>f4','>f4','>u2'],'offsets':[0,4,8,13],'itemsize':17})
    raw=bytearray(42)
    a=np.ndarray((2,1),dtype=dtype,buffer=raw,strides=(21,17))
    a['x']=[[1],[2]];a['y']=0;a['z']=3;a['confidence']=75
    fields=[NS(name=k,offset=v,datatype=t,count=1) for k,v,t in
            [('x',0,7),('y',4,7),('z',8,7),('confidence',13,4)]]
    msg=NS(data=raw,width=1,height=2,point_step=17,row_step=21,is_bigendian=True,fields=fields)
    result=cloud_arrays(msg)
    np.testing.assert_array_equal(result['x'],[1,2])
    np.testing.assert_array_equal(result['confidence'],[75,75])


def stamped(ns):
    return NS(header=NS(stamp=NS(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000)))


def test_pairing_rejects_missing_frame_and_duplicate_timestamp():
    sync=PairSynchronizer(5_000_000)
    sync.add('image',stamped(0))
    sync.add('image',stamped(100_000_000))
    pairs=sync.add('cloud',stamped(101_685_342))
    assert len(pairs)==1
    assert sync.stats['image_unmatched']==1
    assert sync.add('cloud',stamped(101_685_342))==[]
    assert sync.stats['cloud_non_increasing']==1
