"""Offline geometry, model parsing, sequencing and cancellation regressions."""
import copy
import base64
import json
import math
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace

import cv2
import numpy as np

from nova2_grasp.controller import Controller
from nova2_grasp.geometry import target_geometry, vertical_tilt, map_pixel
from nova2_grasp.vision import json_reply, validate_detection, YoloDetector, Camera


class FakePrimitives:
    def __init__(self):
        self.calls = []
        self.pose = [200., -200., 500., -90., 0., 0.]
        self.joints = [1., 2., 3., 4., 5., 6.]
        self.holding = False
        self.enabled = True
        self.fail_descent = False
        self.never_hold = False
        self.moves = []

    def call(self, group, name, **args):
        self.calls.append((group, name, copy.deepcopy(args)))
        if group == "arm":
            if name == "get_state":
                return dict(mode=5 if self.enabled else 4, enabled=self.enabled, error=False,
                    collision=False, paused=False, fault="", running=False, queue_running=False,
                    feedback_user=0, feedback_tool=0, dashboard_errors=[],
                    joint_velocity_deg_s=[0]*6,
                    joints_deg=self.joints.copy(), pose_mm_deg=self.pose.copy())
            if name == "inverse_kinematics":
                return dict(joints_deg=[10, 20, 30, 40, 50, 60])
            if name == "move_pose":
                self.moves.append(copy.deepcopy(args))
                if self.fail_descent and args['pose_mm_deg'][2] == 100:
                    return dict(run_id="failed", state="FAILED")
                self.pose = args['pose_mm_deg'].copy()
                self.joints = [10, 20, 30, 40, 50, 60]
                return dict(run_id=str(len(self.moves)), state="SUCCEEDED")
            if name == "move_joints":
                self.moves.append(copy.deepcopy(args));self.joints = args['joints_deg'].copy()
                self.pose = [200., -200., 500., -90., 0., 0.]
                return dict(run_id=str(len(self.moves)), state="SUCCEEDED")
            if name == "stop":
                return dict(stopped=True)
        elif group == "gripper":
            if name == "get_state":
                return dict(holding=self.holding, moving=False, error_flags=0, write_fault="")
            if name == "open":
                self.holding = False;return dict(accepted=True)
            if name == "grip":
                self.holding = not self.never_hold;return dict(accepted=True)
        raise AssertionError((group, name, args))


class FakeCamera:
    def capture(self, calibration):
        image = np.zeros((600, 800, 3), np.uint8)
        return dict(image=image, jpeg=cv2.imencode('.jpg', image)[1].tobytes(),
                    camera_key="fixture", frame_id="1", captured_at=time.time())


class FakeDetector:
    def detect(self, frame, description):
        return dict(found=True, label=description, confidence=.95, backend="vlm",
                    top_corners_px=[[200,200],[400,200],[400,300],[200,300]])


def calibration():
    return dict(format="nova2-planar-handeye-v1", accepted=True, frames=dict(user=0,tool=0),
        camera_key="fixture", image_size=[800,600], pixel_to_xy=[[.1,0,140],[0,.1,-300],[0,0,1]],
        valid_pixel_polygon=[[0,0],[799,0],[799,599],[0,599]], samples=9,
        max_error_mm=.5, z_mm=100., orientation_rxryrz_deg=[180.,0.,0.])


class Fixture(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory();self.root = Path(self.temp.name)
        (self.root/'calibration.json').write_text(json.dumps(calibration()))
        self.p = FakePrimitives()
        (self.root/'ready.json').write_text(json.dumps(dict(user=0,tool=0,
            pose_mm_deg=self.p.pose,joints_deg=self.p.joints)))
        (self.root/'place.json').write_text(json.dumps({
            '白盒子': {'xy_mm': [190, -260], 'release_height_mm': 150},
            '高托盘': {'xy_mm': [195, -265], 'release_height_mm': 250},
        }))
        self.c = Controller(dict(calibration_path='calibration.json',ready_pose_path='ready.json',
            place_locations_path='place.json',
            runtime_path='runtime.json',artifacts_dir='artifacts',jaw_alignment_confirmed=True,
            gripper_timeout_s=1),self.root,self.p,FakeCamera(),dict(vlm=FakeDetector(),yolo=FakeDetector()))

    def tearDown(self):
        if self.c.active():self.c.stop();self.c.thread.join(3)
        self.temp.cleanup()

    def run_task(self, action='pick', **kwargs):
        self.c.start(action, 'green block', **kwargs);self.c.thread.join(4)
        self.assertFalse(self.c.thread.is_alive())
        return self.c.task_status()

    def test_detection_and_ik_are_read_only(self):
        plan=self.c.detect_object('green block',check_ik=True)
        self.assertTrue(plan['ik_checked']);self.assertTrue(Path(plan['annotated_image']).is_file())
        self.assertEqual(plan['grasp_pose_mm_deg'][2],100)
        self.assertEqual(plan['hover_pose_mm_deg'][2],300)
        self.assertTrue(all(name=='inverse_kinematics' for _,name,_ in self.p.calls))

    def use_side_pair_detection(self):
        self.c.detectors["vlm"] = SimpleNamespace(detect=lambda *_: dict(
            found=True, label="carrot", confidence=.95, backend="vlm",
            grasp_geometry="side_pair", side_points_px=[[300,200],[300,300]]))

    def test_side_pair_plan_and_pick_use_the_same_taught_z_as_flat_top(self):
        flat_plan = self.c.detect_object("green block", check_ik=True)
        self.use_side_pair_detection()
        plan = self.c.detect_object("carrot", check_ik=True)
        self.assertEqual(plan["geometry"]["xy_mm"], [170,-275])
        self.assertAlmostEqual(plan["geometry"]["closing_width_mm"], 10)
        self.assertEqual(plan["grasp_pose_mm_deg"][2], flat_plan["grasp_pose_mm_deg"][2])
        self.assertEqual(plan["hover_pose_mm_deg"][2], flat_plan["hover_pose_mm_deg"][2])
        self.assertEqual(len(plan["ik_solutions"]), 3)
        self.assertFalse(self.p.moves)
        self.assertEqual(self.run_task("pick", plan_id=plan["plan_id"])["state"], "SUCCEEDED")
        self.assertEqual([move["pose_mm_deg"][2] for move in self.p.moves if "pose_mm_deg" in move],
                         [300,300,100,300])

    def test_side_pair_hover_cycle_uses_taught_z(self):
        self.use_side_pair_detection()
        plan = self.c.detect_object("carrot", check_ik=True)
        self.assertEqual(self.run_task("approach", plan_id=plan["plan_id"])["state"], "SUCCEEDED")
        self.p.moves.clear()
        self.assertEqual(self.run_task("cycle_from_hover")["state"], "SUCCEEDED")
        self.assertEqual([move["pose_mm_deg"][2] for move in self.p.moves if "pose_mm_deg" in move],
                         [100,300,100,300])

    def test_full_pick_short_axis_vertical_descent_and_hold_at_hover(self):
        state=self.run_task()
        self.assertEqual(state['state'],'SUCCEEDED',state)
        self.assertEqual([m.get('motion','joints') for m in self.p.moves],['MovJ','MovL','MovL','MovL'])
        align,down,lift=self.p.moves[1:4]
        self.assertEqual(align['pose_mm_deg'][:2],down['pose_mm_deg'][:2])
        self.assertEqual(align['pose_mm_deg'][3:],down['pose_mm_deg'][3:])
        self.assertEqual(lift['pose_mm_deg'],align['pose_mm_deg'])
        self.assertEqual(down['pose_mm_deg'][2],100)
        self.assertLess(vertical_tilt(down['pose_mm_deg']),1e-9)
        self.assertEqual(self.p.pose, lift['pose_mm_deg'])
        self.assertTrue(state['result']['at_hover'])
        self.assertTrue(state['result']['gripper']['holding'])
        writes=[name for group,name,_ in self.p.calls if group=='gripper' and name!='get_state']
        self.assertEqual(writes,['open','grip'])

    def test_pick_releases_at_configured_xy_and_height_after_arrival(self):
        for name, expected in (
            ('白盒子', [[190, -260, 300], [190, -260, 250]]),
            ('高托盘', [[170, -275, 350], [195, -265, 350]]),
        ):
            with self.subTest(destination=name):
                self.p.pose = self.c.ready['pose_mm_deg'].copy()
                self.p.calls.clear()
                self.p.moves.clear()
                state = self.run_task('pick_and_place', destination=name)
                self.assertEqual(state['state'], 'SUCCEEDED', state)
                lift, *transfer = self.p.moves[3:]
                self.assertEqual([m['pose_mm_deg'][:3] for m in transfer], expected)
                self.assertTrue(all(m['motion'] == 'MovL' and
                    m['pose_mm_deg'][3:] == lift['pose_mm_deg'][3:] for m in transfer))
                writes = [(group, action, args) for group, action, args in self.p.calls
                          if action in ('open', 'grip', 'move_pose')]
                self.assertEqual(writes[-1][:2], ('gripper', 'open'))
                self.assertEqual(writes[-2][2]['pose_mm_deg'], transfer[-1]['pose_mm_deg'])
                self.assertEqual(state['result']['destination'], name)
                self.assertEqual(state['result']['release_pose_mm_deg'][:3], expected[-1])
                self.assertFalse(state['result']['gripper']['holding'])

    def test_unknown_destination_and_unreachable_release_do_not_start_pick(self):
        with self.assertRaisesRegex(ValueError, '未知'):
            self.c.start('pick_and_place', 'green block', destination='不存在')
        with self.assertRaisesRegex(ValueError, '未指定'):
            self.c.start('pick_and_place', 'green block')
        call = self.p.call

        def reject_release(group, name, **args):
            if name == 'inverse_kinematics' and args['pose_mm_deg'][:3] == [190, -260, 250]:
                raise ValueError('fixture release IK rejected')
            return call(group, name, **args)

        with patch.object(self.p, 'call', side_effect=reject_release):
            state = self.run_task('pick_and_place', destination='白盒子')
        self.assertEqual(state['state'], 'FAILED')
        self.assertEqual(state['stage'], 'CHECK_DESTINATION_IK')
        self.assertFalse(self.p.moves)
        self.assertFalse(any(name in ('open', 'grip') for _, name, _ in self.p.calls))

    def test_failed_destination_move_keeps_object_in_gripper(self):
        call = self.p.call

        def fail_transfer(group, name, **args):
            if name == 'move_pose' and args['pose_mm_deg'][:2] == [190, -260]:
                return dict(run_id='failed', state='FAILED')
            return call(group, name, **args)

        with patch.object(self.p, 'call', side_effect=fail_transfer):
            state = self.run_task('pick_and_place', destination='白盒子')
        self.assertEqual(state['state'], 'FAILED')
        self.assertEqual(state['stage'], 'MOVE_ABOVE_DESTINATION')
        self.assertTrue(self.p.holding)
        self.assertEqual([name for _, name, _ in self.p.calls if name == 'open'], ['open'])

    def test_pick_moves_to_ready_pose_and_waits_before_capturing_fresh_image(self):
        plan = self.c.detect_object("green block")
        ready_pose = self.c.ready["pose_mm_deg"]
        # A matching joint readout alone must not skip the ready pose check.
        self.p.pose = [240, -200, 500, -90, 0, 0]
        parent_call = self.p.call
        capture = self.c.camera.capture
        events = []

        def call(group, name, **args):
            if group == "arm" and name == "move_pose" and args["pose_mm_deg"] == ready_pose:
                events.append("submitted")
                self.assertEqual(args["motion"], "MovJ")
                return {"run_id": "ready", "state": "RUNNING"}
            if group == "arm" and name == "motion_status":
                self.assertEqual(args["run_id"], "ready")
                self.p.pose = ready_pose.copy()
                events.append("arrived")
                return {"run_id": "ready", "state": "SUCCEEDED"}
            return parent_call(group, name, **args)

        def capture_at_ready(cal):
            self.assertEqual(events[:2], ["submitted", "arrived"])
            self.assertEqual(self.p.pose, ready_pose)
            events.append("captured")
            return capture(cal)

        with patch.object(self.p, "call", side_effect=call), patch.object(
                self.c.camera, "capture", side_effect=capture_at_ready):
            state = self.run_task("pick", plan_id=plan["plan_id"])
        self.assertEqual(state["state"], "SUCCEEDED", state)
        self.assertNotEqual(state["result"]["plan"]["plan_id"], plan["plan_id"])

    def test_pick_stops_before_capture_if_ready_pose_was_not_reached(self):
        self.p.pose = [240, -200, 500, -90, 0, 0]
        with patch.object(self.c, "_move", return_value=None), patch.object(
                self.c.camera, "capture") as capture:
            state = self.run_task("pick")
        self.assertEqual(state["stage"], "READY")
        self.assertEqual(state["state"], "FAILED")
        self.assertIn("尚未到达就绪位", state["message"])
        capture.assert_not_called()
        self.assertFalse(any(name in ("open", "grip") for _, name, _ in self.p.calls))

    def test_approach_does_not_descend_or_change_jaws(self):
        self.c.set_jaw_alignment(0,False)
        state=self.run_task('approach')
        self.assertEqual(state['state'],'SUCCEEDED',state)
        self.assertEqual(len(self.p.moves),2)
        self.assertTrue(all(m['pose_mm_deg'][2]==300 for m in self.p.moves))
        self.assertFalse(any(g=='gripper' and n in ('open','grip') for g,n,_ in self.p.calls))
        with self.assertRaisesRegex(ValueError,'尚未核对'):self.c.start('pick','green block')

    def test_failed_descent_never_closes_or_returns(self):
        self.p.fail_descent=True
        state=self.run_task()
        self.assertEqual(state['state'],'FAILED')
        self.assertFalse(any(n=='grip' for _,n,_ in self.p.calls))
        self.assertEqual(len(self.p.moves),3)
        self.assertTrue(any(n=='stop' for _,n,_ in self.p.calls))

    def test_no_holding_feedback_never_lifts(self):
        self.p.never_hold=True
        state=self.run_task()
        self.assertEqual(state['state'],'FAILED')
        self.assertEqual(len(self.p.moves),3)
        self.assertIn('未确认稳定夹持',state['message'])

    def test_disabled_arm_does_not_auto_enable_or_move(self):
        self.p.enabled=False;state=self.run_task()
        self.assertEqual(state['state'],'FAILED');self.assertFalse(self.p.moves)
        self.assertFalse(any(n=='enable' for _,n,_ in self.p.calls))

    def test_occupied_jaws_not_opened(self):
        self.p.holding=True;state=self.run_task()
        self.assertEqual(state['state'],'FAILED');self.assertFalse(self.p.moves)
        self.assertFalse(any(n=='open' for _,n,_ in self.p.calls))

    def test_alignment_persists_and_invalidates_plans(self):
        plan=self.c.detect_object('green block');self.c.set_jaw_alignment(90,True)
        self.assertFalse(self.c.plans)
        self.assertEqual(json.loads((self.root/'runtime.json').read_text())['jaw_axis_offset_deg'],90)
        with self.assertRaisesRegex(ValueError,'失效'):self.c.start('pick',plan_id=plan['plan_id'])

    def test_expired_or_modified_calibration_rejected_without_motion(self):
        plan=self.c.detect_object('green block');self.c.plans[plan['plan_id']]['expires_at']=0
        with self.assertRaisesRegex(ValueError,'过期'):self.c.start('pick',plan_id=plan['plan_id'])
        (self.root/'calibration.json').write_text('{}')
        with self.assertRaisesRegex(ValueError,'已变化'):self.c.detect_object('green block')
        self.assertFalse(self.p.moves)

    def test_camera_mode_change_invalidates_existing_plan(self):
        plan=self.c.detect_object('green block')
        original=self.c.camera.capture
        def capture(cal):
            frame=original(cal);frame['camera_key']='changed-camera';return frame
        self.c.camera.capture=capture
        with self.assertRaisesRegex(ValueError,'相机模式'):self.c.check_plan(plan)
        self.assertFalse(self.p.moves)

    def test_release_has_no_arm_motion(self):
        self.p.holding=True;state=self.run_task('release')
        self.assertEqual(state['state'],'SUCCEEDED');self.assertFalse(self.p.moves);self.assertFalse(self.p.holding)

    def test_low_home_raises_first_and_keeps_jaws(self):
        self.p.pose=[180,-270,100,180,0,90];self.p.joints=[10,20,30,40,50,60];self.p.holding=True
        state=self.run_task('home')
        self.assertEqual(state['state'],'SUCCEEDED')
        self.assertEqual(self.p.moves[0]['pose_mm_deg'],[180,-270,300,180,0,90])
        self.assertTrue(self.p.holding)

    def test_cycle_from_verified_hover_places_back_then_returns(self):
        self.p.pose=[170,-275,300,180,0,-50];self.p.joints=[10,20,30,40,50,60]
        state=self.run_task('cycle_from_hover')
        self.assertEqual(state['state'],'SUCCEEDED',state)
        poses=[m['pose_mm_deg'] for m in self.p.moves if 'pose_mm_deg' in m]
        self.assertEqual([p[2] for p in poses],[100,300,100,300])
        self.assertTrue(all(p[:2]==[170,-275] and p[3:]==[180,0,-50] for p in poses))
        self.assertFalse(self.p.holding);self.assertEqual(self.p.joints,self.c.ready['joints_deg'])
        writes=[n for g,n,_ in self.p.calls if g=='gripper' and n in ('open','grip')]
        self.assertEqual(writes,['open','grip','open'])

    def test_cycle_requires_verified_hover_height(self):
        state=self.run_task('cycle_from_hover')
        self.assertEqual(state['state'],'FAILED');self.assertFalse(self.p.moves)
        self.assertFalse(any(n in ('open','grip') for _,n,_ in self.p.calls))

    def test_user_speed_ceiling_accepts_80_and_rejects_81(self):
        self.assertEqual(Controller({**self.c.config,'velocity_percent':80},self.root,self.p,FakeCamera(),self.c.detectors).config['velocity_percent'],80)
        with self.assertRaises(ValueError):Controller({**self.c.config,'velocity_percent':81},self.root,self.p)

    def test_cancel_during_detection_never_moves_or_opens(self):
        entered=threading.Event();release=threading.Event()
        original=self.c.detectors['vlm'].detect
        def detect(*args):entered.set();release.wait(2);return original(*args)
        self.c.detectors['vlm'].detect=detect
        self.c.start('pick','green block');self.assertTrue(entered.wait(1))
        self.c.stop();release.set();self.c.thread.join(3)
        self.assertEqual(self.c.task_status()['state'],'CANCELED');self.assertFalse(self.p.moves)
        self.assertFalse(any(n in ('open','grip') for _,n,_ in self.p.calls))


class CameraPrimitiveTests(unittest.TestCase):
    def payload(self):
        frame=FakeCamera().capture(calibration())
        return dict(jpeg_base64=base64.b64encode(frame['jpeg']).decode(),camera_key='fixture',
                    frame_id='123',mime_type='image/jpeg',age_s=0.)

    def capture(self,data):
        calls=[]
        def call(group,name):calls.append((group,name));return data
        result=Camera({'camera_max_frame_age_s':3.},SimpleNamespace(call=call)).capture(calibration())
        self.assertEqual(calls,[('camera','snapshot')])
        return result

    def test_camera_snapshot_uses_primitive_not_http(self):
        with patch('requests.get',side_effect=AssertionError('HTTP camera access forbidden')):
            frame=self.capture(self.payload())
        self.assertEqual(frame['image'].shape,(600,800,3));self.assertEqual(frame['frame_id'],'123')

    def test_camera_rejects_stale_invalid_or_wrong_calibration_before_detection(self):
        for update in (dict(age_s=4),dict(age_s=float('nan')),dict(age_s=True),dict(camera_key='wrong'),
                       dict(jpeg_base64='??'),dict(jpeg_base64=''),dict(mime_type='image/png')):
            with self.subTest(update=update),self.assertRaises(ValueError):self.capture(self.payload()|update)
        image=np.zeros((100,100,3),np.uint8)
        with self.assertRaisesRegex(ValueError,'分辨率'):
            self.capture(self.payload()|dict(jpeg_base64=base64.b64encode(cv2.imencode('.jpg',image)[1]).decode()))


class GeometryTests(unittest.TestCase):
    def test_short_axis_in_robot_coordinates_not_pixel_box(self):
        cal=calibration();corners=[[200,200],[400,200],[400,300],[200,300]]
        geo=target_geometry(cal,corners,800,600)
        self.assertTrue(np.allclose(geo['xy_mm'],[170,-275]))
        self.assertAlmostEqual(geo['closing_width_mm'],10)
        self.assertAlmostEqual(abs(geo['yaw_deg']),90)
        cal['pixel_to_xy']=[[0,-.4,300],[.1,0,-300],[0,0,1]]
        geo=target_geometry(cal,corners,800,600)
        self.assertAlmostEqual(geo['closing_width_mm'],20)
        self.assertAlmostEqual(abs(geo['closing_axis_xy'][1]),1)

    def test_offset_rotates_tool_yaw_without_changing_xy(self):
        corners=[[200,200],[400,200],[400,300],[200,300]]
        a=target_geometry(calibration(),corners,800,600,0)
        b=target_geometry(calibration(),corners,800,600,90)
        self.assertEqual(a['xy_mm'],b['xy_mm']);self.assertAlmostEqual(b['yaw_deg'],0)

    def test_square_has_no_unique_short_axis(self):
        geo=target_geometry(calibration(),[[200,200],[400,200],[400,400],[200,400]],800,600,reference_yaw_deg=10)
        self.assertTrue(geo['square_ambiguous']);self.assertAlmostEqual(geo['yaw_deg'],0)

    def test_reject_outside_calibration_and_degenerate_face(self):
        with self.assertRaisesRegex(ValueError,'覆盖'):map_pixel(calibration(),[900,300])
        for points in ([[1,1],[1,1],[2,2],[3,3]], [[0,0],[100,0],[100,float('nan')],[0,100]]):
            with self.assertRaises(ValueError):target_geometry(calibration(),points,800,600)

    def test_face_can_straddle_coverage_but_commanded_center_must_be_inside(self):
        cal=calibration();cal['valid_pixel_polygon']=[[250,0],[799,0],[799,599],[250,599]]
        geo=target_geometry(cal,[[200,200],[400,200],[400,300],[200,300]],800,600)
        self.assertTrue(np.allclose(geo['center_pixel'],[300,250]))
        with self.assertRaisesRegex(ValueError,'覆盖'):
            target_geometry(cal,[[100,200],[300,200],[300,300],[100,300]],800,600)

    def test_absolute_vertical_allows_world_yaw(self):
        for yaw in (-180,-90,0,90,180):self.assertLess(vertical_tilt([0,0,0,180,0,yaw]),1e-9)
        self.assertAlmostEqual(vertical_tilt([0,0,0,170,0,90]),10)


class DetectorTests(unittest.TestCase):
    def test_model_json_contract(self):
        self.assertTrue(json_reply('```json\n{"found":true}\n```')['found'])
        for value in ('{"found":"true"}', '[]', 'not JSON'):
            with self.assertRaises(ValueError):json_reply(value)

    def test_detection_confidence_and_corners(self):
        frame=FakeCamera().capture(calibration());d=FakeDetector().detect(frame,'green block')
        self.assertEqual(len(validate_detection(d,frame,.6)['top_corners_px']),4)
        d['confidence']=.2
        with self.assertRaises(ValueError):validate_detection(d,frame,.6)
        d['confidence']=.9;d['top_corners_px'][0]=[-1,0]
        with self.assertRaises(ValueError):validate_detection(d,frame,.6)

    def test_yolo_missing_weights_never_silently_falls_back(self):
        with self.assertRaisesRegex(ValueError,'权重'):
            YoloDetector(dict(yolo_model_path='')).detect({},'green block')

    def test_yolo_obb_adapter_and_class_alias(self):
        class Tensor:
            def __init__(self,value):self.value=np.array(value)
            def cpu(self):return self
            def numpy(self):return self.value
        result=SimpleNamespace(names={0:'green_cube'},obb=SimpleNamespace(
            xyxyxyxy=Tensor([[[200,200],[400,200],[400,300],[200,300]]]),
            conf=Tensor([.91]),cls=Tensor([0])))
        model=SimpleNamespace(predict=lambda *a,**k:[result])
        with tempfile.TemporaryDirectory() as directory:
            weights=Path(directory)/'fixture.pt';weights.touch()
            with patch.dict('sys.modules',{'ultralytics':SimpleNamespace(YOLO=lambda _:model)}):
                detector=YoloDetector(dict(yolo_model_path=str(weights),min_confidence=.6,
                    yolo_class_aliases_json='{"绿色积木":"green_cube"}'))
                frame=FakeCamera().capture(calibration())
                detection=detector.detect(frame,'绿色积木')
                self.assertTrue(detection['found']);self.assertEqual(detection['backend'],'yolo')
                self.assertEqual(len(validate_detection(detection,frame,.6)['top_corners_px']),4)
                self.assertFalse(detector.detect(frame,'red_cube')['found'])
                result.obb=None
                with self.assertRaisesRegex(ValueError,'水平框'):detector.detect(frame,'绿色积木')


if __name__=='__main__':unittest.main()
