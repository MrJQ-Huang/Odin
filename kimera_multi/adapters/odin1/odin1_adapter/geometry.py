import cv2
import numpy as np

from .calibration import intrinsics


def cloud_arrays(msg):
    """Read PointCloud2 including unaligned fields, endianness and row padding."""
    types = {1:'i1', 2:'u1', 3:'i2', 4:'u2', 5:'i4', 6:'u4', 7:'f4', 8:'f8'}
    endian = '>' if msg.is_bigendian else '<'
    fields = {f.name: f for f in msg.fields}
    required = ('x', 'y', 'z', 'confidence')
    if not all(name in fields and fields[name].count == 1 for name in required):
        raise ValueError('Odin cloud requires scalar x, y, z, confidence fields')
    dtype = np.dtype({'names': list(fields),
                      'formats': [endian+types[f.datatype] if f.count == 1
                                  else (endian+types[f.datatype], f.count) for f in fields.values()],
                      'offsets': [f.offset for f in fields.values()], 'itemsize': msg.point_step})
    rows = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=memoryview(msg.data),
                      strides=(msg.row_step, msg.point_step)).reshape(-1)
    return {name: rows[name] for name in fields}


class RgbdProjector:
    def __init__(self, config):
        self.config = config
        self.K = intrinsics(config)
        self.T_camera_imu = config['T_camera_lidar'] @ np.linalg.inv(config['T_imu_lidar'])
        self.width, self.height = config['output_width'], config['output_height']
        camera = config['camera']
        u, v = np.meshgrid(np.arange(self.width), np.arange(self.height))
        x, y = (u-self.K[0,2])/self.K[0,0], (v-self.K[1,2])/self.K[1,1]
        radius = np.hypot(x, y)
        theta = np.arctan(radius)
        distorted_theta = theta.copy()
        for power in range(2, 8):
            distorted_theta += camera[f'k{power}'] * theta**power
        scale = np.divide(distorted_theta, radius, out=np.ones_like(radius), where=radius > 1e-12)
        self.map_x = (camera['A11']*x*scale + camera.get('A12',0)*y*scale + camera['u0']).astype(np.float32)
        self.map_y = (camera['A22']*y*scale + camera['v0']).astype(np.float32)
        self.rgb_valid = ((self.map_x >= 0) & (self.map_x < camera['image_width']-1)
                          & (self.map_y >= 0) & (self.map_y < camera['image_height']-1))

    def rectify(self, bgr):
        camera = self.config['camera']
        if bgr is None or bgr.shape != (camera['image_height'], camera['image_width'], 3):
            raise ValueError('RGB dimensions do not match device calibration')
        return cv2.remap(bgr, self.map_x, self.map_y, cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT)

    def project(self, arrays, cloud_stamp_ns, image_stamp_ns, poses=None):
        cfg = self.config
        xyz = np.column_stack([arrays[name] for name in ('x','y','z')]).astype(float)
        valid = (np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz,axis=1) > .05)
                 & (arrays['confidence'] >= cfg['confidence_threshold']))
        if cfg['deskew'] == 'odometry':
            if poses is None or 'offset_time' not in arrays:
                raise ValueError('Odometry deskew requires poses and per-point offset_time')
            offsets = arrays['offset_time'].astype(float)
            valid &= np.isfinite(offsets) & (offsets >= 0) & (offsets <= .2)
            times = cloud_stamp_ns + np.rint(offsets[valid]*1e9).astype(np.int64)
            # LiDAR(t) -> IMU(t) -> world -> IMU(image) -> LiDAR(image).
            points_imu = apply_transform(cfg['T_imu_lidar'], xyz[valid])
            # Odin scan rows share acquisition times. Interpolate each unique
            # time once, while retaining each point's exact offset.
            unique_times,index = np.unique(times,return_inverse=True)
            rotations, translations = poses.interpolate(unique_times, cfg['max_pose_gap_ms']*1e6)
            points_world = np.einsum('nij,nj->ni',rotations[index],points_imu) + translations[index]
            R, t = poses.interpolate(np.array([image_stamp_ns]), cfg['max_pose_gap_ms']*1e6)
            points_imu_image = np.einsum('ni,ij->nj',points_world-t[0],R[0])
            points_camera = apply_transform(self.T_camera_imu, points_imu_image)
        else:
            points_camera = apply_transform(cfg['T_camera_lidar'], xyz[valid])
        z = points_camera[:,2]
        good = np.isfinite(points_camera).all(axis=1) & (z >= cfg['min_depth_m']) & (z <= cfg['max_depth_m'])
        points_camera, z = points_camera[good], z[good]
        uv = np.rint(points_camera[:,:2]/z[:,None] * [self.K[0,0],self.K[1,1]]
                     + [self.K[0,2],self.K[1,2]]).astype(int)
        depth = np.full((self.height,self.width),np.inf,np.float32)
        direct = np.zeros_like(depth,np.uint8)
        inside = (uv[:,0]>=0)&(uv[:,0]<self.width)&(uv[:,1]>=0)&(uv[:,1]<self.height)
        # Do not let out-of-FOV measurements splat into the image.
        uv, z = uv[inside], z[inside]
        direct[uv[:,1],uv[:,0]] = 255
        np.minimum.at(depth,(uv[:,1],uv[:,0]),z)
        radius = cfg['splat_radius']
        if radius:
            # Equivalent to a nearest-surface splat at every measurement,
            # implemented as one z-buffer pass followed by a local minimum.
            depth = cv2.erode(depth,np.ones((2*radius+1,2*radius+1),np.uint8),
                             borderType=cv2.BORDER_CONSTANT,borderValue=float('inf'))
        finite = np.isfinite(depth)
        local_min = cv2.erode(np.where(finite,depth,1e6),np.ones((3,3),np.uint8))
        local_max = cv2.dilate(np.where(finite,depth,0),np.ones((3,3),np.uint8))
        edges = (local_max-local_min) > cfg['edge_threshold_m']
        depth[~finite | edges | ~self.rgb_valid] = 0
        direct[~self.rgb_valid] = 0
        return depth, direct


def apply_transform(matrix, xyz):
    # Small 3-D transforms: avoid spawning a BLAS thread team for each scan.
    return np.einsum('ij,nj->ni',matrix[:3,:3],xyz) + matrix[:3,3]


class PoseUnavailable(ValueError):
    pass


class PoseSeries:
    """Linear translation and shortest-arc quaternion interpolation, no extrapolation."""
    def __init__(self, entries):
        values = sorted(entries, key=lambda row: row[0])
        self.times = np.array([row[0] for row in values],np.int64)
        self.positions = np.array([row[1] for row in values],float).reshape(-1,3)
        self.quaternions = np.array([row[2] for row in values],float).reshape(-1,4)
        if len(self.times) < 2 or np.any(np.diff(self.times) <= 0):
            raise PoseUnavailable('Need two or more strictly increasing odometry poses')
        norms = np.linalg.norm(self.quaternions,axis=1)
        if not np.isfinite(self.positions).all() or not np.isfinite(norms).all() or np.any(norms < 1e-6):
            raise ValueError('Invalid odometry pose')
        self.quaternions /= norms[:,None]

    def interpolate(self, times, max_gap_ns):
        times = np.asarray(times,np.int64)
        if len(times) == 0:
            return np.empty((0,3,3)),np.empty((0,3))
        if times.min() < self.times[0] or times.max() > self.times[-1]:
            raise PoseUnavailable(f'Odometry does not bracket requested {times.min()*1e-9:.6f}..{times.max()*1e-9:.6f} s; available {self.times[0]*1e-9:.6f}..{self.times[-1]*1e-9:.6f} s')
        right = np.clip(np.searchsorted(self.times,times,side='right'),1,len(self.times)-1)
        left = right-1
        gap = self.times[right]-self.times[left]
        if np.any((gap > max_gap_ns) & (times != self.times[left]) & (times != self.times[right])):
            raise PoseUnavailable('Odometry interpolation crosses a recording gap')
        fraction = ((times-self.times[left])/gap)[:,None]
        translation = self.positions[left]*(1-fraction)+self.positions[right]*fraction
        q0,q1 = self.quaternions[left].copy(),self.quaternions[right].copy()
        dot = np.sum(q0*q1,axis=1)
        q1[dot<0] *= -1
        dot = np.clip(np.abs(dot),0,1)
        angle = np.arccos(dot)[:,None]
        sine = np.sin(angle)
        a = np.divide(np.sin((1-fraction)*angle),sine,out=1-fraction.copy(),where=sine>1e-6)
        b = np.divide(np.sin(fraction*angle),sine,out=fraction.copy(),where=sine>1e-6)
        q = a*q0+b*q1
        q /= np.linalg.norm(q,axis=1)[:,None]
        x,y,z,w = q.T
        R = np.empty((len(times),3,3))
        R[:,0,0]=1-2*(y*y+z*z); R[:,0,1]=2*(x*y-z*w); R[:,0,2]=2*(x*z+y*w)
        R[:,1,0]=2*(x*y+z*w); R[:,1,1]=1-2*(x*x+z*z); R[:,1,2]=2*(y*z-x*w)
        R[:,2,0]=2*(x*z-y*w); R[:,2,1]=2*(y*z+x*w); R[:,2,2]=1-2*(x*x+y*y)
        return R,translation
