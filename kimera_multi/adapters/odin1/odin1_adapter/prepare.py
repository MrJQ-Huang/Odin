import argparse
import hashlib
import json
import shutil
from pathlib import Path

import yaml

from .calibration import intrinsics, load_config


class OpenCVDumper(yaml.SafeDumper):
    pass


OpenCVDumper.add_representer(list,lambda dumper,value:
    dumper.represent_sequence('tag:yaml.org,2002:seq',value,flow_style=True))


def opencv_yaml(data):
    return '%YAML:1.0\n---\n'+yaml.dump(data,Dumper=OpenCVDumper,sort_keys=False,width=100000)


def prepare(calib, output, template=None, config_path=None):
    package = Path(__file__).resolve().parents[1]
    repo = package.parents[1]
    config_path = config_path or package/'config/adapter.yaml'
    template = template or repo/'params/Odin1'
    output = Path(output)
    if output.exists():
        raise ValueError(f'Refusing to overwrite calibration directory: {output}')
    output.mkdir(parents=True)
    try:
        shutil.copyfile(calib,output/'calibration.yaml')
        settings = yaml.safe_load(Path(config_path).read_text())
        settings['calibration_file'] = 'calibration.yaml'
        (output/'adapter.yaml').write_text(yaml.safe_dump(settings,sort_keys=False))
        config = load_config(output/'adapter.yaml')
        shutil.copytree(template,output/'params')
        K = intrinsics(config)
        camera = {'camera_id':'left_cam','T_BS':{'rows':4,'cols':4,'data':config['T_imu_camera'].flatten().tolist()},
            'rate_hz':config['camera_rate_hz'], 'resolution':[config['output_width'],config['output_height']],
            'camera_model':'pinhole','intrinsics':[float(K[0,0]),float(K[1,1]),float(K[0,2]),float(K[1,2])],
            'distortion_model':'radtan','distortion_coefficients':[0.,0.,0.,0.],
            'virtual_baseline':0.1,'min_depth':config['min_depth_m'],'max_depth':config['max_depth_m'],
            'depth_to_meters':1.,'is_registered':1}
        (output/'params/LeftCameraParams.yaml').write_text(opencv_yaml(camera))
        path = output/'params/ImuParams.yaml'
        imu = yaml.safe_load(path.read_text().replace('%YAML:1.0',''))
        imu['rate_hz'] = config['imu_rate_hz']
        path.write_text(opencv_yaml(imu).replace('---\n','---\n# Noise parameters are initial tuning values, not an Odin Allan calibration.\n',1))
        metadata = {'source_calibration':str(Path(calib).resolve()),
                    'source_sha256':hashlib.sha256(Path(calib).read_bytes()).hexdigest(),
                    'matrix_convention':'p_A = T_A_B p_B; Kimera T_BS = T_imu_camera',
                    'T_imu_lidar_source':'Explicit adapter configuration; default is the vendor wiki section 5.2 fixed matrix',
                    'imu_noise_calibrated':False,
                    'note':'Firmware WIWC matrices are not adopted automatically; local driver TF conventions differ from the wiki.'}
        (output/'generation.json').write_text(json.dumps(metadata,indent=2))
    except Exception:
        shutil.rmtree(output)
        raise
    return output


def main():
    parser = argparse.ArgumentParser(description='Prepare Odin camera/IMU configuration for the Kimera RGB-D pipeline')
    parser.add_argument('--calib',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--config',type=Path,help='Override adapter settings, including T_imu_lidar')
    parser.add_argument('--template',type=Path,help='Kimera parameter template directory')
    args = parser.parse_args()
    print(prepare(args.calib,args.output,args.template,args.config).resolve())


if __name__ == '__main__':
    main()
