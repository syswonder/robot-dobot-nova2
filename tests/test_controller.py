from __future__ import annotations

import time
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from fake_robot import FakeRobot
from nova2_arm.controller import Controller, quaternion_to_euler
from nova2_arm.protocol import ControllerError, Feedback, parse_reply


class MotionTests(unittest.TestCase):
    def setUp(self):
        self.robot = FakeRobot()
        self.controller = Controller(self.robot.config)
        self.controller.start()

    def tearDown(self):
        self.controller.close()
        self.robot.close()

    def test_state_enable_disable_no_startup_motion(self):
        self.assertFalse(self.controller.state()["enabled"])
        self.assertFalse(any(c.startswith(("EnableRobot", "JointMovJ", "MovJ", "MovL")) for c in self.robot.commands))
        self.assertEqual(self.controller.enable()["mode"], 5)
        self.assertEqual(self.controller.disable()["mode"], 4)

    def test_rejects_motion_when_disabled(self):
        with self.assertRaisesRegex(ControllerError, "enabled, idle"):
            self.controller.move_joints([1, -20, 40, 0, 40, 0])
        self.assertFalse(any(c.startswith("JointMovJ") for c in self.robot.commands))

    def test_disable_remains_available_when_alarm_masks_idle_mode(self):
        self.controller.enable()
        self.robot.error = True
        deadline = time.monotonic() + 1
        while self.controller.feedback.snapshot()["mode"] != 9 and time.monotonic() < deadline:
            time.sleep(.02)
        result = self.controller.disable()
        self.assertFalse(result["enabled"])
        self.assertTrue(result["error"])
        self.assertEqual(result["mode"], 9)

    def test_joint_motion_requires_measured_arrival(self):
        self.controller.enable()
        run = self.controller.move_joints([3, -20, 40, 0, 40, 0], 5)
        self.assertEqual(run["state"], "RUNNING")
        with self.assertRaises(ControllerError):
            self.controller.move_joints([4, -20, 40, 0, 40, 0])
        result = self.controller.motion_status(run["run_id"], 2)
        self.assertEqual(result["state"], "SUCCEEDED", result)
        self.assertEqual(self.controller.state()["joints_deg"][0], 3)
        command = next(c for c in self.robot.commands if c.startswith("JointMovJ"))
        self.assertIn("SpeedJ=5,AccJ=10", command)
        self.assertIsNone(result["command_id"])

    def test_enabled_idle_queue_engine_does_not_block_motion(self):
        self.controller.enable()
        packet = self.controller.feedback._sample
        self.assertEqual(int(packet["run_queued_cmd"]), 1)
        self.assertTrue(self.controller.state()["queue_running"])
        run = self.controller.move_joints([1, -20, 40, 0, 40, 0], 2)
        self.assertEqual(self.controller.motion_status(run["run_id"], 2)["state"], "SUCCEEDED")
        self.assertTrue(self.controller.stop()["stopped"])

    def test_pose_motion_uses_correct_frame_and_native_v3_format(self):
        self.controller.enable()
        run = self.controller.move_pose([110, -200, 300, 180, 0, 90], "MovL", 5)
        result = self.controller.motion_status(run["run_id"], 2)
        self.assertEqual(result["state"], "SUCCEEDED", result)
        command = next(c for c in self.robot.commands if c.startswith("MovL"))
        self.assertIn("User=0,Tool=0", command)
        self.assertIn("SpeedL=5,AccL=10", command)
        self.assertIn("TCPSpeedEnd()", self.robot.commands)
        self.assertIn("SetArmOrientation(1,1,-1,0)", self.robot.commands)
        self.assertEqual(self.controller.state()["hand_type"], [1, 1, -1, 0])
        self.assertTrue(any(c.startswith("InverseSolution(") and ",0,0,1,{" in c for c in self.robot.commands))

    def test_quaternion_motion_and_validation(self):
        self.controller.enable()
        q = Rotation.from_euler("xyz", [20, 30, 40], degrees=True).as_quat().tolist()
        run = self.controller.move_quaternion([110, -200, 300], q, "MovJ", 5)
        self.assertEqual(self.controller.motion_status(run["run_id"], 2)["state"], "SUCCEEDED")
        np.testing.assert_allclose(self.robot.pose[3:], [20, 30, 40], atol=1e-5)
        self.assertTrue(any(c.startswith("JointMovJ(") for c in self.robot.commands))
        with self.assertRaises(ValueError):
            self.controller.move_quaternion([1, 2, 3], [0, 0, 0, 0])

    def test_stop_cancels_motion_without_waiting_for_target(self):
        self.controller.enable()
        self.robot.duration = 10
        run = self.controller.move_joints([8, -20, 40, 0, 40, 0])
        self.controller.stop()
        self.assertEqual(self.controller.motion_status(run["run_id"])["state"], "CANCELED")
        self.assertTrue(any(c == "ResetRobot()" for c in self.robot.commands))
        self.assertEqual(self.robot.joints[0], 0)

    def test_validation_prevents_bad_commands(self):
        for joints in [[0] * 5, [0, 0, 180, 0, 0, 0], [float("nan")] * 6, [True] * 6]:
            with self.assertRaises(ValueError):
                self.controller.move_joints(joints)
        with self.assertRaises(ValueError):
            self.controller.move_joints([0] * 6, 21)
        with self.assertRaises(ValueError):
            self.controller.move_pose([0] * 6, "servo")
        self.assertFalse(self.robot.commands)

    def test_controller_rejection_is_not_reported_as_success(self):
        self.controller.enable()
        self.robot.reject_next_motion = True
        with self.assertRaises(ControllerError):
            self.controller.move_joints([3, -20, 40, 0, 40, 0])
        self.assertEqual(self.controller.motion_status()["state"], "FAILED")

    def test_lost_ack_is_not_replayed_and_motion_is_locked(self):
        self.controller.enable()
        self.robot.drop_next_motion_ack = True
        with self.assertRaisesRegex(ControllerError, "outcome may be unknown"):
            self.controller.move_joints([3, -20, 40, 0, 40, 0])
        self.assertEqual(self.controller.motion_status()["state"], "UNKNOWN")
        with self.assertRaisesRegex(ControllerError, "inspection"):
            self.controller.move_joints([4, -20, 40, 0, 40, 0])
        self.assertEqual(sum(c.startswith("JointMovJ") for c in self.robot.commands), 1)
        self.controller.stop()

    def test_stale_feedback_fails_motion_and_never_claims_arrival(self):
        self.controller.enable()
        self.robot.duration = 10
        run = self.controller.move_joints([8, -20, 40, 0, 40, 0])
        self.robot.halt_feedback = True
        result = self.controller.motion_status(run["run_id"], 2)
        self.assertEqual(result["state"], "UNKNOWN", result)
        self.assertTrue(any(c == "ResetRobot()" for c in self.robot.commands))

    def test_idle_at_wrong_target_is_failure(self):
        self.controller.enable()
        self.robot.wrong_target = True
        run = self.controller.move_joints([8, -20, 40, 0, 40, 0])
        result = self.controller.motion_status(run["run_id"], 3)
        self.assertEqual(result["state"], "FAILED", result)

    def test_motion_deadline_requests_stop(self):
        self.controller.enable()
        self.controller.config["motion_timeout_s"] = .3
        self.robot.duration = 10
        run = self.controller.move_joints([8, -20, 40, 0, 40, 0])
        result = self.controller.motion_status(run["run_id"], 2)
        self.assertEqual(result["state"], "FAILED", result)
        self.assertIn("deadline", result["message"])


class ProtocolTests(unittest.TestCase):
    def test_replies_and_orientation(self):
        self.assertEqual(parse_reply("0,{1,2,3},GetAngle();"), "1,2,3")
        self.assertEqual(parse_reply("0,{42},MovJ(joint={1,2,3,4,5,6});"), "42")
        with self.assertRaisesRegex(ControllerError, "TCP/IP mode"):
            parse_reply("-2,{not tcp mode},RobotMode();")
        with self.assertRaises(ControllerError):
            parse_reply("garbage")
        q = Rotation.from_euler("xyz", [10, 20, 30], degrees=True).as_quat().tolist()
        np.testing.assert_allclose(quaternion_to_euler(q), [10, 20, 30], atol=1e-9)
        with self.assertRaisesRegex(ValueError, "singularity"):
            quaternion_to_euler(Rotation.from_euler("xyz", [0, 90, 0], degrees=True).as_quat().tolist())

    def test_v4_joint_motion_matches_command_id(self):
        robot = FakeRobot("v4")
        c = Controller(robot.config)
        try:
            c.start(); c.enable()
            run = c.move_joints([5, -20, 40, 0, 40, 0], 5)
            result = c.motion_status(run["run_id"], 2)
            self.assertEqual(result["state"], "SUCCEEDED", result)
            self.assertEqual(result["command_id"], 1)
            self.assertTrue(any(c.startswith("MovJ(joint={") for c in robot.commands))
        finally:
            c.close(); robot.close()

    def test_invalid_feedback_layout_is_rejected(self):
        robot = FakeRobot()
        robot.bad_feedback = True
        f = Feedback("127.0.0.1", robot.feedback.server_address[1], .15, "v3")
        try:
            with self.assertRaises(ControllerError):
                f.start()
        finally:
            f.close(); robot.close()


if __name__ == "__main__":
    unittest.main()
