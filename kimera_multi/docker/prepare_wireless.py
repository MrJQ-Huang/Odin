"""Assemble wireless extension context without bags or host installations."""
import argparse
import hashlib
import json
import pathlib
import shutil
import subprocess

ROOT = pathlib.Path(__file__).resolve().parents[1]
CONTEXT = ROOT / 'artifacts/docker/wireless-context'

def copy(source, target):
    shutil.copytree(source, target, dirs_exist_ok=True, symlinks=True,
                    ignore=shutil.ignore_patterns('.git', '__pycache__', '*.egg-info', '.pytest_cache'))

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--odin-source', type=pathlib.Path, default=ROOT.parent / 'odin_ros_driver-main')
    args=parser.parse_args()
    if not (args.odin_source / 'src/host_sdk_sample.cpp').exists():
        parser.error('--odin-source must point to the vendor odin_ros_driver-main directory')
    CONTEXT.mkdir(parents=True, exist_ok=True)
    specs=json.loads((ROOT / 'docker/wireless.repos').read_text())['repositories']
    manifest={'repositories':specs, 'driver_files':{}}
    for name,spec in specs.items():
        cache=ROOT / 'artifacts/sdk_frontend_deps' / name
        if not (cache / '.git').exists():
            subprocess.run(['git','clone','--no-checkout',spec['url'],str(cache)],check=True)
        head=subprocess.check_output(['git','-C',str(cache),'rev-parse','HEAD'],text=True).strip()
        if head != spec['version']:
            subprocess.run(['git','-C',str(cache),'fetch','--depth','1','origin',spec['version']],check=True)
            subprocess.run(['git','-C',str(cache),'checkout','--detach',spec['version']],check=True)
        if subprocess.check_output(['git','-C',str(cache),'status','--porcelain'],text=True).strip():
            raise RuntimeError('Dependency cache modified: '+str(cache))
        dest=CONTEXT / 'deps' / name
        if dest.exists(): shutil.rmtree(dest)
        copy(cache,dest)
    driver=CONTEXT / 'driver'
    if driver.exists(): shutil.rmtree(driver)
    driver.mkdir()
    for name in ('CMakeLists.txt','package.xml','include','src','srv','lib','config'):
        src=args.odin_source / name
        if src.is_dir(): copy(src,driver/name)
        else: shutil.copy2(src,driver/name)
    for p in sorted(driver.rglob('*')):
        if p.is_file(): manifest['driver_files'][str(p.relative_to(driver))]=hashlib.sha256(p.read_bytes()).hexdigest()
    package=CONTEXT / 'kimera_multi'
    if package.exists(): shutil.rmtree(package)
    package.mkdir()
    for name in ('package.xml','CMakeLists.txt'): shutil.copy2(ROOT/name,package/name)
    for name in ('launch','params','adapters','src','include'): copy(ROOT/name,package/name)
    copy(ROOT / 'docker',CONTEXT / 'docker')
    (CONTEXT / 'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(CONTEXT)

if __name__=='__main__': main()
