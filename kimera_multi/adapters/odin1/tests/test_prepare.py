import cv2
import numpy as np
import pytest
import yaml

from odin1_adapter.calibration import load_config
from odin1_adapter.prepare import prepare


def test_generated_camera_body_transform_and_opencv_yaml(tmp_path):
    # A nontrivial camera rotation/translation catches using T_CL in place of
    # its inverse when generating the camera-to-body pose.
    T_camera_lidar=np.array([[0,-1,0,.2],[1,0,0,.3],[0,0,1,.4],[0,0,0,1.]])
    calib={'cam_num':1,'Tcl_0':T_camera_lidar.flatten().tolist(),
           'cam_0':dict(cam_model='FishPoly',image_width=1600,image_height=1296,A11=700.,A22=710.,
                        A12=0.,u0=800.,v0=648.,**{f'k{i}':0. for i in range(2,8)})}
    source=tmp_path/'calib.yaml';source.write_text(yaml.safe_dump(calib))
    output=prepare(source,tmp_path/'prepared')
    config=load_config(output/'adapter.yaml')
    camera=yaml.safe_load((output/'params/LeftCameraParams.yaml').read_text().replace('%YAML:1.0',''))
    T_body_camera=np.array(camera['T_BS']['data']).reshape(4,4)
    np.testing.assert_allclose(T_body_camera@[.2,1.3,2.4,1],[.97337,.03447,2.02174,1],atol=1e-8)
    assert camera['depth_to_meters']==1.
    assert camera['resolution']==[640,518]
    storage=cv2.FileStorage(str(output/'params/LeftCameraParams.yaml'),cv2.FILE_STORAGE_READ)
    assert storage.isOpened()
    assert storage.getNode('intrinsics').at(0).real()==280.
    storage.release()
    imu_storage=cv2.FileStorage(str(output/'params/ImuParams.yaml'),cv2.FILE_STORAGE_READ)
    assert imu_storage.isOpened()
    assert imu_storage.getNode('rate_hz').real()==400.8
    imu_storage.release()
    pipeline=yaml.safe_load((output/'params/PipelineParams.yaml').read_text().replace('%YAML:1.0',''))
    assert pipeline['frontend_type']==2
    with pytest.raises(ValueError,match='overwrite'):
        prepare(source,output)
