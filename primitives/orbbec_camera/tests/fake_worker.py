"""Deterministic SDK pipe fixture; no USB or network access."""
import argparse
import json
import os
import struct
import sys
import threading
import time

import cv2
import numpy as np


def metadata(c):
    return dict(backend='orbbec',device='fixture',serial='fixture',width=c['color_width'],height=c['color_height'],
                format='OBFormat.MJPG',processing='undistorted_color_same_K_v1',
                K=[[600.,0.,400.],[0.,600.,300.],[0.,0.,1.]],D=[0.]*8)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--config-json',required=True)
    p.add_argument('--mode',default='normal')
    a = p.parse_args()
    c = json.loads(a.config_json)
    stopped = threading.Event()
    def stop_on_eof():
        os.read(sys.stdin.fileno(),1)
        stopped.set()
    threading.Thread(target=stop_on_eof,daemon=True).start()
    image = np.zeros((c['color_height'],c['color_width'],3),np.uint8)
    jpeg = cv2.imencode('.jpg',image)[1].tobytes()
    m = metadata(c)
    if a.mode=='wrong_size':m['width']+=1
    if a.mode=='raw':m['processing']='raw_color_v1'
    if a.mode=='bad_jpeg':jpeg=b'not a jpeg'
    if a.mode=='error':m={'error':'fixture device unplugged'}
    if a.mode=='bad_length':
        sys.stdout.buffer.write(b'NCAM'+struct.pack('<II',999999,0));sys.stdout.buffer.flush()
        return
    while not stopped.is_set():
        if a.mode!='silent':
            header=json.dumps(m|dict(captured_at=time.time())).encode()
            sys.stdout.buffer.write(b'NCAM'+struct.pack('<II',len(header),len(jpeg))+header+jpeg)
            sys.stdout.buffer.flush()
        if a.mode in ('once','exit'):
            if a.mode=='exit':return
            stopped.wait()
            return
        stopped.wait(.05)


if __name__=='__main__':main()
