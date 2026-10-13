"""All T_A_B matrices map coordinates in frame B into frame A."""
from pathlib import Path

import numpy as np
import yaml


def transform(value, name):
    matrix = np.asarray(value, dtype=float).reshape(4, 4)
    if not np.isfinite(matrix).all() or not np.allclose(matrix[3], [0, 0, 0, 1]):
        raise ValueError(f'{name}: invalid homogeneous transform')
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-4) or not np.isclose(np.linalg.det(rotation), 1, atol=2e-4):
        raise ValueError(f'{name}: rotation must be orthonormal and right handed')
    return matrix


def load_config(filename):
    path = Path(filename).resolve()
    config = yaml.safe_load(path.read_text())
    if not isinstance(config, dict):
        raise ValueError('Adapter configuration must be a mapping')
    calibration = Path(config['calibration_file']).expanduser()
    if not calibration.is_absolute():
        calibration = path.parent / calibration
    data = yaml.safe_load(calibration.read_text())
    if data['cam_0']['cam_model'] != 'FishPoly':
        raise ValueError('Only the Odin FishPoly camera model is supported')
    config['camera'] = data['cam_0']
    config['T_camera_lidar'] = transform(data['Tcl_0'], 'Tcl_0')
    config['T_imu_lidar'] = transform(config['T_imu_lidar'], 'T_imu_lidar')
    config['T_imu_camera'] = config['T_imu_lidar'] @ np.linalg.inv(config['T_camera_lidar'])
    config['calibration_file'] = str(calibration.resolve())
    width, height = config['output_width'], config['output_height']
    if not isinstance(width, int) or not isinstance(height, int) or min(width, height) < 1:
        raise ValueError('Output dimensions must be positive integers')
    camera = config['camera']
    if min(camera['A11'], camera['A22'], camera['image_width'], camera['image_height']) <= 0:
        raise ValueError('Invalid camera dimensions or focal lengths')
    if not 0 < config['min_depth_m'] < config['max_depth_m']:
        raise ValueError('Require 0 < min_depth_m < max_depth_m')
    if config['deskew'] not in ('none', 'odometry'):
        raise ValueError('deskew must be none or odometry')
    if config['splat_radius'] not in (0, 1, 2):
        raise ValueError('splat_radius must be 0, 1 or 2')
    if min(config['sync_tolerance_ms'], config['max_pose_gap_ms'], config['edge_threshold_m']) <= 0:
        raise ValueError('Synchronization, pose gap and edge thresholds must be positive')
    config['robot_name'] = config['robot_name'].strip('/')
    if not config['robot_name'] or '/' in config['robot_name']:
        raise ValueError('robot_name must be one namespace component')
    return config


def intrinsics(config):
    camera = config['camera']
    sx = config['output_width'] / camera['image_width']
    sy = config['output_height'] / camera['image_height']
    # Rectify into a zero-skew pinhole camera, shared by RGB and depth.
    return np.array([[camera['A11']*sx, 0., camera['u0']*sx],
                     [0., camera['A22']*sy, camera['v0']*sy], [0., 0., 1.]])
