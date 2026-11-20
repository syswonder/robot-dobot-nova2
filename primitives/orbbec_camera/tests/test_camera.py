import base64
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest

PKG=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PKG))
from orbbec_usb.controller import Controller,DEFAULTS,camera_key
from fake_worker import metadata


class CameraTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.controllers=[]

    def tearDown(self):
        for c in self.controllers:c.close()
        self.temp.cleanup()

    def camera(self,mode='normal',**config):
        c=DEFAULTS|dict(color_width=80,color_height=60,startup_timeout_s=2.)|config
        command=[sys.executable,str(PKG/'tests/fake_worker.py'),'--config-json',json.dumps(c),'--mode',mode]
        controller=Controller(c,self.temp.name,command=command)
        self.controllers.append(controller)
        return controller

    def test_real_pipe_snapshot_metadata_and_graceful_release(self):
        c=self.camera();c.start()
        self.assertTrue(c.state()['connected'])
        frame=c.snapshot()
        self.assertEqual(frame['camera_key'],camera_key(metadata(c.config)))
        self.assertTrue(base64.b64decode(frame['jpeg_base64']).startswith(b'\xff\xd8'))
        self.assertEqual(c.camera_info()['metadata']['width'],80)
        self.assertFalse(c.camera_info()['depth_enabled'])
        process=c.process
        c.close();c.close()
        self.assertEqual(process.returncode,0)
        self.assertFalse(c.state()['connected'])
        with self.assertRaises(RuntimeError):c.snapshot()
        c.start();self.assertTrue(c.state()['connected'])

    def test_pin_calibration_and_serial_before_activation_succeeds(self):
        for config in (dict(expected_camera_key='wrong'),dict(serial_number='wrong')):
            c=self.camera(**config)
            with self.assertRaisesRegex(RuntimeError,'differ|serial mismatch'):c.start()
            self.assertIsNone(c.process)

    def test_wrong_image_mode_jpeg_or_length_never_activates(self):
        for mode in ('wrong_size','raw','bad_jpeg','bad_length','error'):
            with self.subTest(mode=mode):
                c=self.camera(mode)
                with self.assertRaises(RuntimeError):c.start()
                self.assertFalse(c.state()['connected'])

    def test_stale_frame_refused_even_if_process_alive(self):
        c=self.camera('once',max_frame_age_s=.1);c.start();time.sleep(.15)
        self.assertFalse(c.state()['connected'])
        with self.assertRaisesRegex(RuntimeError,'stale'):c.snapshot()

    def test_no_frame_startup_timeout_cleans_up_worker(self):
        c=self.camera('silent',startup_timeout_s=.2)
        with self.assertRaises(TimeoutError):c.start()
        self.assertIsNone(c.process)

    def test_exited_worker_does_not_serve_cached_frame(self):
        c=self.camera('exit')
        try:c.start()
        except RuntimeError:return
        c.process.wait(timeout=2)
        deadline=time.monotonic()+2
        while c.state()['connected'] and time.monotonic()<deadline:time.sleep(.01)
        self.assertFalse(c.state()['connected'])

    def test_invalid_config_rejected_without_starting_worker(self):
        for config in (dict(backend='http'),dict(color_width=True),dict(max_frame_age_s=float('nan')),
                       dict(color_format='INVALID'),dict(publish_fps=0),dict(serial_number=1),dict(unknown=1)):
            with self.assertRaises(ValueError):Controller(config,self.temp.name)


if __name__=='__main__':unittest.main()
