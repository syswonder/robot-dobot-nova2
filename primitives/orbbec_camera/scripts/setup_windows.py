"""Private Windows runtime, no PATH/global Python/driver changes."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from orbbec_usb import wsl

VERSIONS = {'numpy':'2.5.5','opencv-python-headless':'5.1.0.95','pyorbbecsdk2':'2.1.3','av':'19.0.2'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed-runtime',type=Path,help='Optional verified embedded runtime to COPY once, not a live dependency')
    args = parser.parse_args()
    local = wsl.run(subprocess.check_output, ['powershell.exe','-NoProfile','-NonInteractive','-Command',
        '[Environment]::GetFolderPath("LocalApplicationData")'],text=True).strip()
    runtime = Path(subprocess.check_output(['wslpath','-u',local],text=True).strip())/'Robonix-Nova2-Orbbec'
    python = runtime/'python/python.exe'
    stamp = runtime/'sdk-versions.json'
    if not python.exists() and args.seed_runtime:
        if not (args.seed_runtime/'python/python.exe').exists():raise RuntimeError('Invalid seed runtime')
        runtime.mkdir(parents=True,exist_ok=True)
        shutil.copytree(args.seed_runtime/'python',runtime/'python',dirs_exist_ok=True)
    if not python.exists():
        cache = ROOT/'rbnx-build/downloads'
        cache.mkdir(parents=True,exist_ok=True)
        archive = cache/'python-3.13.17-embed-amd64.zip'
        if not archive.exists():
            urllib.request.urlretrieve('https://www.python.org/ftp/python/3.13.17/python-3.13.17-embed-amd64.zip',archive)
        (runtime/'python').mkdir(parents=True,exist_ok=True)
        with zipfile.ZipFile(archive) as z:z.extractall(runtime/'python')
        (runtime/'python/python313._pth').write_text('python313.zip\n.\nLib/site-packages\n')
    check = 'import json,importlib.metadata as m;import cv2,numpy,pyorbbecsdk;print(json.dumps({k:m.version(k) for k in '+repr(list(VERSIONS))+'}))'
    try:
        versions = json.loads(wsl.run(subprocess.check_output, [str(python),'-c',check],text=True).strip().splitlines()[-1])
    except (subprocess.CalledProcessError,ValueError):versions = {}
    if versions != VERSIONS:
        subprocess.run(['uv','pip','install','--python-version','3.13','--python-platform','x86_64-pc-windows-msvc',
            '--only-binary',':all:','--no-deps','--link-mode','copy','--target',str(runtime/'python/Lib/site-packages'),
            *[f'{k}=={v}' for k,v in VERSIONS.items()]],check=True)
        versions = json.loads(wsl.run(subprocess.check_output, [str(python),'-c',check],text=True).strip().splitlines()[-1])
        if versions != VERSIONS:raise RuntimeError('Private SDK runtime verification failed')
    stamp.write_text(json.dumps(VERSIONS,indent=2)+'\n')
    (ROOT/'rbnx-build').mkdir(parents=True,exist_ok=True)
    (ROOT/'rbnx-build/windows-runtime.txt').write_text(str(runtime)+'\n')
    print('Private Windows Orbbec SDK runtime verified:',runtime)


if __name__ == '__main__':main()
