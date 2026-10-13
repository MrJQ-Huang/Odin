"""Assemble a small build context; never send recordings or the venv to Docker."""
import concurrent.futures
import hashlib
import json
import pathlib
import shutil
import subprocess
import urllib.request
import zipfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
CACHE = ROOT / 'artifacts/docker'
CONTEXT = CACHE / 'context'
VOCAB_SHA256 = '0e4efc3be84e05c18faec402b88b145c37d94fa41f3eb39999b56ae20f6f2da2'


def checkout(item):
    name, spec = item
    dest = CACHE / 'src' / name
    if not (dest / '.git').is_dir():
        dest.mkdir(parents=True, exist_ok=True)
        subprocess.run(['git', 'init', '-q', str(dest)], check=True)
        subprocess.run(['git', '-C', str(dest), 'remote', 'add', 'origin', spec['url']], check=True)
    head = subprocess.run(['git', '-C', str(dest), 'rev-parse', 'HEAD'],
                          capture_output=True, text=True).stdout.strip()
    if head != spec['version']:
        subprocess.run(['git', '-C', str(dest), 'fetch', '--depth', '1', 'origin', spec['version']], check=True)
        subprocess.run(['git', '-C', str(dest), 'checkout', '--detach', spec['version']], check=True)
    if subprocess.check_output(['git', '-C', str(dest), 'status', '--porcelain'], text=True).strip():
        raise RuntimeError(f'Cached dependency is modified: {dest}; refusing to overwrite it')
    print(f'{name}: {spec["version"]}', flush=True)
    shutil.copytree(dest, CONTEXT / 'deps' / name, dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('.git', '__pycache__'), symlinks=True)


def main():
    CACHE.mkdir(parents=True, exist_ok=True)
    if CONTEXT.exists():
        shutil.rmtree(CONTEXT)
    CONTEXT.mkdir()
    # JSON is valid .repos/YAML and needs no host Python packages.
    repos = json.loads((ROOT / 'docker/odin1.repos').read_text())['repositories']
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        list(pool.map(checkout, repos.items()))
    archive = CACHE / 'ORBvoc.zip'
    if not archive.exists():
        temporary = archive.with_suffix('.partial')
        urllib.request.urlretrieve('https://www.dropbox.com/s/lyo0qgbdxn6eg6o/ORBvoc.zip?dl=1', temporary)
        temporary.rename(archive)
    if hashlib.sha256(archive.read_bytes()).hexdigest() != VOCAB_SHA256:
        raise RuntimeError(f'Vocabulary checksum mismatch: {archive}')
    target = CONTEXT / 'vocabulary'
    target.mkdir()
    with zipfile.ZipFile(archive) as source:
        for name in ('ORBvoc.yml', 'LICENSE.txt', 'License-gpl.txt'):
            with source.open('ORBvoc/' + name) as inp, (target / name).open('wb') as out:
                shutil.copyfileobj(inp, out)
    shutil.copytree(ROOT / 'docker', CONTEXT / 'docker',
                    ignore=shutil.ignore_patterns('volume', '__pycache__'))
    package = CONTEXT / 'kimera_multi'
    package.mkdir()
    for name in ('package.xml', 'CMakeLists.txt'):
        shutil.copy2(ROOT / name, package / name)
    for name in ('launch', 'params', 'adapters', 'src', 'include'):
        shutil.copytree(ROOT / name, package / name,
                        ignore=shutil.ignore_patterns('__pycache__', '*.egg-info', '.pytest_cache'))
    print(f'Build context: {CONTEXT}', flush=True)


if __name__ == '__main__':
    main()
