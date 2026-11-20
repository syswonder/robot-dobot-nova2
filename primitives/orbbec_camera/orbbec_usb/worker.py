"""SDK worker: private stdin lifetime, framed JPEG/metadata on stdout."""
from __future__ import annotations

import argparse
import json
import os
import struct
import sys
import threading
import time

import cv2
import numpy as np


def frames(c, stopped):
    from pyorbbecsdk import Context, Pipeline, Config, OBSensorType, OBFormat
    context = Context()
    devices = context.query_devices()
    if c['serial_number']:
        device = devices.get_device_by_serial_number(c['serial_number'])
    else:
        if devices.get_count() != 1:
            raise RuntimeError('Expected exactly one camera; configure serial_number')
        device = devices.get_device_by_index(0)
    info = device.get_device_info()
    pipeline = Pipeline(device)
    profiles = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
    profile = profiles.get_video_stream_profile(c['color_width'], c['color_height'],
                                              getattr(OBFormat, c['color_format']), c['color_fps'])
    config = Config()
    config.enable_stream(profile)
    # Require real intrinsics; never silently substitute raw/another projection.
    intrinsic, dist = profile.get_intrinsic(), profile.get_distortion()
    matrix = np.array([[intrinsic.fx, 0, intrinsic.cx], [0, intrinsic.fy, intrinsic.cy], [0, 0, 1.]], float)
    distortion = np.array([getattr(dist, k, 0.) for k in ('k1','k2','p1','p2','k3','k4','k5','k6')], float)
    if not np.isfinite(matrix).all() or not np.isfinite(distortion).all() or intrinsic.fx <= 0 or intrinsic.fy <= 0:
        raise RuntimeError('Invalid color intrinsics')
    meta = dict(backend='orbbec', device=info.get_name(), serial=info.get_serial_number(),
                processing='undistorted_color_same_K_v1', K=matrix.tolist(), D=distortion.tolist(),fps=profile.get_fps())
    pipeline.start(config)
    previous = 0.
    try:
        while not stopped.is_set():
            bundle = pipeline.wait_for_frames(1000)
            frame = bundle.get_color_frame() if bundle else None
            if frame is None or time.monotonic()-previous < 1/c['publish_fps']:
                continue
            width, height, fmt = frame.get_width(), frame.get_height(), frame.get_format()
            data = np.frombuffer(frame.get_data(), dtype=np.uint8)
            if fmt == OBFormat.RGB:
                image = cv2.cvtColor(data.reshape(height,width,3), cv2.COLOR_RGB2BGR)
            elif fmt == OBFormat.BGR:
                image = data.reshape(height,width,3)
            elif fmt == OBFormat.MJPG:
                image = cv2.imdecode(data, cv2.IMREAD_COLOR)
            elif fmt == OBFormat.YUYV:
                image = cv2.cvtColor(data.reshape(height,width,2), cv2.COLOR_YUV2BGR_YUY2)
            else:
                raise RuntimeError('Unsupported color format: '+str(fmt))
            if image is None or [width,height] != [c['color_width'],c['color_height']]:
                raise RuntimeError('Unexpected color image size')
            image = cv2.undistort(image, matrix, distortion)
            ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY,c['jpeg_quality']])
            if not ok:
                raise RuntimeError('JPEG encode failed')
            yield meta | dict(width=width,height=height,format=str(fmt),captured_at=time.time()), jpeg.tobytes()
            previous = time.monotonic()
    finally:
        pipeline.stop()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config-json', required=True)
    c = json.loads(parser.parse_args().config_json)
    if os.name == 'nt':
        import msvcrt
        for stream in (sys.stdin, sys.stdout):
            msvcrt.setmode(stream.fileno(), os.O_BINARY)
    output = os.fdopen(os.dup(sys.stdout.fileno()), 'wb', buffering=0)
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    sys.stdout = sys.stderr  # SDK diagnostics must not corrupt the binary pipe.
    stopped = threading.Event()
    def watch_parent():
        os.read(sys.stdin.fileno(),1)  # No buffered-I/O lock during interpreter shutdown.
        stopped.set()
    threading.Thread(target=watch_parent, daemon=True).start()
    def send(meta, jpeg=b''):
        header = json.dumps(meta, ensure_ascii=False).encode()
        output.write(b'NCAM'+struct.pack('<II',len(header),len(jpeg))+header+jpeg)
    try:
        for meta, jpeg in frames(c, stopped):
            send(meta,jpeg)
    except Exception as error:
        send(dict(error=str(error)))
        raise
    finally:
        output.close()


if __name__ == '__main__':
    main()
