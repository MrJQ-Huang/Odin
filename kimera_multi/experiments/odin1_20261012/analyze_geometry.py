#!/usr/bin/python3
"""Check measured LiDAR support for image features using factory calibration."""
import json
from pathlib import Path
import cv2
import numpy as np
import yaml

root=Path(__file__).resolve().parent
calib=yaml.safe_load((root/'runtime/calib.yaml').read_text())
camera=calib['cam_0']; T=np.array(calib['Tcl_0']).reshape(4,4)
data=np.load(root/'static_baseline/cloud_raw_sample.npz')
xyz=np.stack([data[x] for x in ('x','y','z')],axis=1)
valid=np.isfinite(xyz).all(axis=1)&(np.linalg.norm(xyz,axis=1)>0.1)
if 'confidence' in data:
    valid &= data['confidence'] >= 35
points=xyz[valid]@T[:3,:3].T+T[:3,3]
points=points[(points[:,2]>0.1)&(points[:,2]<20)]
img=cv2.imread(str(root/'static_baseline/image_undistorted_00.png'))
scale=640/img.shape[1]; width=640; height=round(img.shape[0]*scale)
img=cv2.resize(img,(width,height),interpolation=cv2.INTER_AREA)
u=(camera['A11']*points[:,0]/points[:,2]+camera['A12']*points[:,1]/points[:,2]+camera['u0'])*scale
v=(camera['A22']*points[:,1]/points[:,2]+camera['v0'])*scale
uv=np.rint(np.stack([u,v],axis=1)).astype(int)
inside=(uv[:,0]>=0)&(uv[:,0]<width)&(uv[:,1]>=0)&(uv[:,1]<height)
uv=uv[inside]; points=points[inside]
depth=np.full((height,width),np.inf,dtype=np.float32)
np.minimum.at(depth,(uv[:,1],uv[:,0]),points[:,2])
mask=np.isfinite(depth).astype(np.uint8)
near=cv2.dilate(mask,np.ones((3,3),np.uint8))
keypoints,descriptors=cv2.ORB_create(nfeatures=2000).detectAndCompute(img,None)
pixels=np.rint([kp.pt for kp in keypoints]).astype(int)
exact=mask[pixels[:,1],pixels[:,0]] if len(pixels) else np.array([])
supported=near[pixels[:,1],pixels[:,0]] if len(pixels) else np.array([])
canvas=img.copy()
values=np.clip(points[:,2]/10*255,0,255).astype(np.uint8)
colors=cv2.applyColorMap(values[:,None],cv2.COLORMAP_TURBO).reshape(-1,3)
for i in range(0,len(uv),3):
    cv2.circle(canvas,tuple(uv[i]),1,tuple(int(x) for x in colors[i]),-1)
cv2.imwrite(str(root/'lidar_rgb_projection.png'),np.concatenate([img,canvas],axis=1))
np.save(root/'measured_depth_640.npy',np.where(np.isfinite(depth),depth,0))
result={'sample_scope':'one static frame; factory calibration, confidence >= 35; no depth completion',
    'width':width,'height':height,'raw_points':len(xyz),'valid_confident_points':int(valid.sum()),
    'points_in_camera_view':len(points),'measured_pixel_fraction':float(mask.mean()),
    'fraction_with_measurement_within_one_pixel':float(near.mean()),
    'orb_features':len(keypoints),'orb_exact_depth_features':int(exact.sum()),
    'orb_features_with_measurement_within_one_pixel':int(supported.sum()),
    'image_header_frame_id':'empty in observed driver output; adapter must set camera optical frame'}
(root/'geometry_summary.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
