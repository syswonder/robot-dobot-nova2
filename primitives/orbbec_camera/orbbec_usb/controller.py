"""Lifecycle-owned SDK process; only fresh, calibrated-size RGB snapshots."""
from __future__ import annotations

import base64
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import threading
import time

from . import wsl


DEFAULTS = dict(backend='auto',serial_number='',color_width=1280,color_height=720,
                color_format='MJPG',color_fps=10,publish_fps=8,jpeg_quality=88,
                startup_timeout_s=30.,max_frame_age_s=3.,expected_camera_key='')


def camera_key(meta):
    # Same fingerprint as the independent service that produced existing calibration.
    keys = ('backend','device','serial','width','height','format','processing','K','D')
    return hashlib.sha256(json.dumps({k:meta.get(k) for k in keys},sort_keys=True).encode()).hexdigest()


class Controller:
    def __init__(self, config, root, command=None):
        unknown = set(config)-set(DEFAULTS)
        if unknown:
            raise ValueError('Unknown camera config: '+', '.join(sorted(unknown)))
        self.config = DEFAULTS | config
        c = self.config
        if c['backend'] not in ('auto','native','windows') or c['color_format'] not in ('MJPG','RGB','BGR','YUYV'):
            raise ValueError('Invalid camera backend or color format')
        for name, lo, hi in [('color_width',1,4096),('color_height',1,4096),('color_fps',1,120),
                              ('publish_fps',1,30),('jpeg_quality',1,100)]:
            v = c[name]
            if isinstance(v,bool) or not isinstance(v,int) or not lo <= v <= hi:
                raise ValueError(f'{name} must be integer {lo}..{hi}')
        for name in ('startup_timeout_s','max_frame_age_s'):
            v = c[name]
            if isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v) or not .1 <= v <= 60:
                raise ValueError(name+' must be finite 0.1..60 seconds')
        for name in ('serial_number','expected_camera_key'):
            if not isinstance(c[name],str):raise ValueError(name+' must be string')
        self.root = Path(root)
        self.command_override = command  # Unit/integration fixtures only, not a deployment config.
        self.condition = threading.Condition()
        self.lifecycle_lock = threading.Lock()
        self.process = None
        self.reader = None
        self.latest = None
        self.counter = 0
        self.error = 'Camera not active'
        self.log = None
        self.stopping = False

    def command(self):
        if self.command_override is not None:return self.command_override
        c = self.config
        windows = c['backend']=='windows' or (c['backend']=='auto' and os.name!='nt' and shutil.which('powershell.exe'))
        worker = self.root/'orbbec_usb/worker.py'
        if windows:
            setting = self.root/'rbnx-build/windows-runtime.txt'
            if not setting.exists():raise RuntimeError('Windows SDK runtime missing: rebuild camera package')
            runtime = Path(setting.read_text().strip())
            target = runtime/'worker.py'
            shutil.copyfile(worker,target)
            executable = str(runtime/'python/python.exe')
            worker = subprocess.check_output(['wslpath','-w',str(target)],text=True).strip()
        else:
            executable = sys.executable
        return [executable,str(worker),'--config-json',json.dumps(c)]

    def start(self):
        with self.lifecycle_lock:
            if self.process and self.process.poll() is None:
                self.frame()
                return
            self._close()
            with self.condition:
                self.latest = None
                self.error = ''
                self.stopping = False
            folder = self.root/'rbnx-build'
            folder.mkdir(parents=True,exist_ok=True)
            self.log = (folder/'camera-worker.log').open('ab')
            try:
                self.process = wsl.run(subprocess.Popen, self.command(), stdin=subprocess.PIPE,
                                       stdout=subprocess.PIPE, stderr=self.log, bufsize=0)
                self.reader = threading.Thread(target=self._read_loop,daemon=True)
                self.reader.start()
                deadline = time.monotonic()+self.config['startup_timeout_s']
                with self.condition:
                    while self.latest is None and not self.error:
                        remaining = deadline-time.monotonic()
                        if remaining <= 0:raise TimeoutError('No fresh camera frame before startup timeout')
                        self.condition.wait(remaining)
                    self.frame()
            except BaseException:
                self._close()
                raise

    def _read_exact(self,n):
        data = bytearray()
        while len(data)<n:
            part = self.process.stdout.read(n-len(data))
            if not part:raise RuntimeError('SDK camera worker exited; check rbnx-build/camera-worker.log')
            data.extend(part)
        return bytes(data)

    def _read_loop(self):
        try:
            while True:
                magic = self._read_exact(4)
                while magic != b'NCAM':magic = magic[1:]+self._read_exact(1)
                size,jpeg_size = struct.unpack('<II',self._read_exact(8))
                if not 0<size<=65536 or jpeg_size>10000000:raise RuntimeError('Invalid camera pipe frame length')
                meta = json.loads(self._read_exact(size))
                jpeg = self._read_exact(jpeg_size)
                self._publish(meta,jpeg)
        except Exception as error:
            with self.condition:
                if not self.stopping:self.error = str(error)
                self.condition.notify_all()

    def _publish(self,meta,jpeg):
        c = self.config
        if 'error' in meta:raise RuntimeError('Orbbec SDK: '+str(meta['error']))
        if [meta.get('width'),meta.get('height')] != [c['color_width'],c['color_height']]:
            raise ValueError('SDK camera resolution mismatch')
        if meta.get('processing')!='undistorted_color_same_K_v1' or not meta.get('K') or not meta.get('D'):
            raise ValueError('SDK worker must provide undistorted image and intrinsics')
        if c['serial_number'] and meta.get('serial')!=c['serial_number']:
            raise ValueError('Camera serial mismatch')
        key = camera_key(meta)
        if c['expected_camera_key'] and key!=c['expected_camera_key']:
            raise ValueError('Camera settings/intrinsics differ from saved calibration')
        if not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9'):
            raise ValueError('Invalid JPEG payload')
        with self.condition:
            self.counter += 1
            self.latest = dict(frame_id=str(self.counter),camera_key=key,metadata=meta,
                               jpeg=jpeg,received_monotonic=time.monotonic(),received_at_unix_s=time.time())
            self.error = ''
            self.condition.notify_all()

    def frame(self):
        with self.condition:
            if self.error:raise RuntimeError(self.error)
            if self.latest is None:raise RuntimeError('No camera frame')
            age = time.monotonic()-self.latest['received_monotonic']
            if age>self.config['max_frame_age_s']:raise RuntimeError('Camera frame stale')
            return self.latest.copy() | dict(age_s=age)

    def state(self):
        with self.condition:
            try:
                f = self.frame()
                return dict(connected=True,error='',frame_id=f['frame_id'],camera_key=f['camera_key'],
                            age_s=f['age_s'],metadata=f['metadata'],source='orbbec_sdk_usb')
            except RuntimeError as error:
                return dict(connected=False,error=str(error),source='orbbec_sdk_usb')

    def snapshot(self):
        f = self.frame()
        return dict(frame_id=f['frame_id'],camera_key=f['camera_key'],metadata=f['metadata'],
                    captured_at_unix_s=f['metadata'].get('captured_at'),age_s=f['age_s'],
                    received_at_unix_s=f['received_at_unix_s'],mime_type='image/jpeg',
                    jpeg_base64=base64.b64encode(f['jpeg']).decode('ascii'))

    def camera_info(self):
        f = self.frame()
        return dict(camera_key=f['camera_key'],metadata=f['metadata'],
                    pixel_coordinates='undistorted color, same K, original resolution',depth_enabled=False)

    def _close(self):
        self.stopping = True
        if self.process:
            if self.process.stdin:
                try:self.process.stdin.close()
                except OSError:pass
            try:self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:self.process.wait(timeout=3)
                except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
            if self.reader:self.reader.join(timeout=2)
            if self.process.stdout:self.process.stdout.close()
        if self.log:self.log.close()
        self.process = self.reader = self.log = None
        with self.condition:
            self.latest = None
            self.error = 'Camera not active'

    def close(self):
        with self.lifecycle_lock:self._close()
